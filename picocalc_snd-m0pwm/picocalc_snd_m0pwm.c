// SPDX-License-Identifier: GPL-2.0
/*
 * PicoCalc M0 delta-sigma audio driver
 * Uses RK3506 Cortex-M0 core to drive GPIO4_B2/B3 from shared memory ring buffer.
 */

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/of_reserved_mem.h>
#include <sound/core.h>
#include <sound/pcm.h>

/* Must match picocalc_m0_fw_pwm/shmem.h and main.c fixed config */
#define M0_AUDIO_MAGIC    0x4D305057U
#define M0_CTRL_PLAY      (1u << 0)
#define M0_CTRL_STOP      0u
#define M0_FMT_U8         0
#define M0_FMT_S16_LE     1
#define M0_HEADER_SIZE    64
/* Fixed M0 config: ALSA does SRC to this rate; ring size must match firmware */
#define M0_FIXED_SAMPLE_RATE_HZ  48000U
#define M0_FIXED_BUF_SIZE        8192U

struct m0_audio_shmem {
	volatile uint32_t magic;
	volatile uint32_t ctrl;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	volatile uint32_t period_bytes;
	volatile uint32_t buf_size;
	volatile uint32_t sample_rate;
	volatile uint32_t channels;
	volatile uint32_t format;
	uint32_t          _reserved[7];
	uint8_t           buffer[];
};

static const struct of_device_id picocalc_snd_m0pwm_dt_ids[] = {
	{ .compatible = "picocalc,snd-m0pwm", },
	{ }
};
MODULE_DEVICE_TABLE(of, picocalc_snd_m0pwm_dt_ids);

struct picocalc_m0pwm {
	struct platform_device *pdev;
	struct snd_card *card;
	struct snd_pcm_substream *substream;
	struct rproc *rproc;
	struct m0_audio_shmem *shmem;
	void __iomem *shmem_io;
	size_t shmem_size;
	uint32_t buf_size;
	spinlock_t lock;
	struct hrtimer timer;
	ktime_t period_ktime;
	uint32_t last_read_idx;
	uint32_t copied_bytes;
	bool running;
};

static struct picocalc_m0pwm *g_m0pwm;

static enum hrtimer_restart m0pwm_timer_cb(struct hrtimer *t)
{
	struct picocalc_m0pwm *m = container_of(t, struct picocalc_m0pwm, timer);
	struct snd_pcm_substream *ss = m->substream;
	struct snd_pcm_runtime *runtime;
	uint32_t read_idx, write_idx, period_bytes, frame_size;
	uint32_t space, to_copy;
	uint8_t *ring;
	const uint8_t *dma_area;

	if (!ss || !m->running)
		return HRTIMER_NORESTART;

	runtime = ss->runtime;
	if (!runtime || !runtime->dma_area)
		return HRTIMER_RESTART;

	period_bytes = m->shmem->period_bytes;
	frame_size = (runtime->channels * (runtime->format == SNDRV_PCM_FORMAT_S16_LE ? 2 : 1));
	ring = (uint8_t *)m->shmem->buffer;
	dma_area = (const uint8_t *)runtime->dma_area;

	read_idx = m->shmem->read_idx;
	write_idx = m->shmem->write_idx;

	/* Free space in ring: up to buf_size - 1 to avoid write_idx == read_idx meaning empty */
	if (write_idx >= read_idx)
		space = m->buf_size - (write_idx - read_idx) - 1;
	else
		space = read_idx - write_idx - 1;

	to_copy = period_bytes;
	if (to_copy > space)
		to_copy = space;
	if (to_copy > 0 && m->copied_bytes + to_copy <= runtime->buffer_size) {
		uint32_t i;
		for (i = 0; i < to_copy; i++) {
			uint32_t wi = (write_idx + i) % m->buf_size;
			uint32_t di = (m->copied_bytes + i) % runtime->buffer_size;
			ring[wi] = dma_area[di];
		}
		m->shmem->write_idx = (write_idx + to_copy) % m->buf_size;
		m->copied_bytes += to_copy;
	}

	/* Period elapsed? */
	if ((uint32_t)(read_idx - m->last_read_idx) >= period_bytes) {
		m->last_read_idx = read_idx;
		snd_pcm_period_elapsed(ss);
	}

	hrtimer_forward(t, hrtimer_cb_get_time(t), m->period_ktime);
	return HRTIMER_RESTART;
}

static int m0pwm_pcm_open(struct snd_pcm_substream *ss)
{
	struct picocalc_m0pwm *m = snd_pcm_substream_chip(ss);

	/* Fixed config: M0 firmware is 48 kHz S16 LE stereo only; ALSA SRC handles other rates */
	ss->runtime->hw = (struct snd_pcm_hardware){
		.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_HALF_DUPLEX,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rates = SNDRV_PCM_RATE_48000,
		.rate_min = M0_FIXED_SAMPLE_RATE_HZ,
		.rate_max = M0_FIXED_SAMPLE_RATE_HZ,
		.channels_min = 2,
		.channels_max = 2,
		.buffer_bytes_max = 32768,
		.period_bytes_min = 1024,
		.period_bytes_max = 8192,
		.periods_min = 2,
		.periods_max = 16,
	};
	m->substream = ss;
	return 0;
}

static int m0pwm_pcm_close(struct snd_pcm_substream *ss)
{
	struct picocalc_m0pwm *m = snd_pcm_substream_chip(ss);

	m->substream = NULL;
	return 0;
}

static int m0pwm_pcm_hw_params(struct snd_pcm_substream *ss,
			       struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw_params));
}

static int m0pwm_pcm_hw_free(struct snd_pcm_substream *ss)
{
	return snd_pcm_lib_free_pages(ss);
}

static int m0pwm_pcm_prepare(struct snd_pcm_substream *ss)
{
	return 0;
}

static int m0pwm_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct picocalc_m0pwm *m = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&m->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (m->running) {
			ret = -EALREADY;
			break;
		}
		m->shmem->magic = M0_AUDIO_MAGIC;
		m->shmem->ctrl = M0_CTRL_PLAY;
		m->shmem->write_idx = 0;
		m->shmem->read_idx = 0;
		m->shmem->period_bytes = frames_to_bytes(runtime, runtime->period_size);
		m->shmem->buf_size = m->buf_size;  /* must be M0_FIXED_BUF_SIZE */
		m->shmem->sample_rate = M0_FIXED_SAMPLE_RATE_HZ;
		m->shmem->channels = 2;
		m->shmem->format = M0_FMT_S16_LE;

		m->last_read_idx = 0;
		m->copied_bytes = 0;
		m->period_ktime = ns_to_ktime(NSEC_PER_SEC / runtime->rate);
		m->running = true;

		ret = rproc_boot(m->rproc);
		if (ret) {
			m->running = false;
			dev_err(&m->pdev->dev, "rproc_boot failed: %d\n", ret);
			break;
		}
		hrtimer_start(&m->timer, m->period_ktime, HRTIMER_MODE_REL);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		if (!m->running)
			break;
		m->running = false;
		hrtimer_cancel(&m->timer);
		m->shmem->ctrl = M0_CTRL_STOP;
		spin_unlock_irqrestore(&m->lock, flags);
		msleep(2);
		rproc_shutdown(m->rproc);
		spin_lock_irqsave(&m->lock, flags);
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&m->lock, flags);
	return ret;
}

static snd_pcm_uframes_t m0pwm_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct picocalc_m0pwm *m = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	uint32_t read_idx = m->shmem->read_idx;
	unsigned int frame_size = frames_to_bytes(runtime, 1);

	return read_idx / frame_size;
}

static const struct snd_pcm_ops m0pwm_pcm_ops = {
	.open    = m0pwm_pcm_open,
	.close   = m0pwm_pcm_close,
	.ioctl   = snd_pcm_lib_ioctl,
	.hw_params = m0pwm_pcm_hw_params,
	.hw_free = m0pwm_pcm_hw_free,
	.prepare = m0pwm_pcm_prepare,
	.trigger = m0pwm_pcm_trigger,
	.pointer = m0pwm_pcm_pointer,
};

static int m0pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct picocalc_m0pwm *m;
	struct reserved_mem *rmem;
	struct device_node *rproc_np;
	struct device_node *mem_np;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->pdev = pdev;
	spin_lock_init(&m->lock);
	hrtimer_init(&m->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	m->timer.function = m0pwm_timer_cb;

	rproc_np = of_parse_phandle(np, "remote-proc", 0);
	if (!rproc_np) {
		dev_err(dev, "missing remote-proc phandle\n");
		return -EINVAL;
	}
	m->rproc = rproc_get_by_phandle(rproc_np->phandle);
	of_node_put(rproc_np);
	if (!m->rproc) {
		dev_err(dev, "rproc_get_by_phandle failed\n");
		return -EPROBE_DEFER;
	}

	mem_np = of_parse_phandle(np, "memory-region", 0);
	if (!mem_np) {
		dev_err(dev, "missing memory-region phandle\n");
		ret = -EINVAL;
		goto put_rproc;
	}
	rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!rmem) {
		dev_err(dev, "missing memory-region / reserved-mem\n");
		ret = -EINVAL;
		goto put_rproc;
	}
	m->shmem_size = rmem->size;
	m->shmem_io = ioremap(rmem->base, rmem->size);
	if (!m->shmem_io) {
		ret = -ENOMEM;
		goto put_rproc;
	}
	m->shmem = (struct m0_audio_shmem *)m->shmem_io;

	if (of_property_read_u32(np, "ring-buffer-bytes", &m->buf_size))
		m->buf_size = M0_FIXED_BUF_SIZE;
	if (m->buf_size != M0_FIXED_BUF_SIZE || m->buf_size > m->shmem_size - M0_HEADER_SIZE)
		m->buf_size = M0_FIXED_BUF_SIZE;

	ret = snd_card_new(dev, SNDRV_DEFAULT_IDX1, "picocalc-m0pwm",
			   THIS_MODULE, 0, &m->card);
	if (ret < 0)
		goto iounmap;

	m->card->private_data = m;
	strscpy(m->card->driver, "picocalc-snd-m0pwm", sizeof(m->card->driver));
	strscpy(m->card->shortname, "PicoCalc M0 Audio", sizeof(m->card->shortname));
	strscpy(m->card->longname, "PicoCalc M0 delta-sigma audio", sizeof(m->card->longname));

	{
		struct snd_pcm *pcm;
		ret = snd_pcm_new(m->card, "M0 PCM", 0, 1, 0, &pcm);
		if (ret < 0)
			goto card_free;
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &m0pwm_pcm_ops);
		snd_pcm_set_drvdata(pcm, m);
		strscpy(pcm->name, "M0 DAC", sizeof(pcm->name));
		snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      dev, 32768, 32768);
	}

	ret = snd_card_register(m->card);
	if (ret < 0)
		goto card_free;

	platform_set_drvdata(pdev, m);
	g_m0pwm = m;
	dev_info(dev, "PicoCalc M0 audio registered (ring %u bytes)\n", m->buf_size);
	return 0;

card_free:
	snd_card_free(m->card);
iounmap:
	iounmap(m->shmem_io);
put_rproc:
	rproc_put(m->rproc);
	return ret;
}

static int m0pwm_remove(struct platform_device *pdev)
{
	struct picocalc_m0pwm *m = platform_get_drvdata(pdev);

	if (!m)
		return 0;
	g_m0pwm = NULL;
	hrtimer_cancel(&m->timer);
	if (m->running) {
		m->shmem->ctrl = M0_CTRL_STOP;
		msleep(2);
		rproc_shutdown(m->rproc);
	}
	snd_card_free(m->card);
	iounmap(m->shmem_io);
	rproc_put(m->rproc);
	return 0;
}

static struct platform_driver picocalc_snd_m0pwm_driver = {
	.driver = {
		.name = "picocalc-snd-m0pwm",
		.of_match_table = picocalc_snd_m0pwm_dt_ids,
	},
	.probe = m0pwm_probe,
	.remove = m0pwm_remove,
};

module_platform_driver(picocalc_snd_m0pwm_driver);

MODULE_AUTHOR("PicoCalc");
MODULE_DESCRIPTION("PicoCalc M0 delta-sigma audio driver");
MODULE_LICENSE("GPL v2");
