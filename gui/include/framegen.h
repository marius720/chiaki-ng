// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#ifndef CHIAKI_FRAMEGEN_H
#define CHIAKI_FRAMEGEN_H

extern "C" {
#include <libplacebo/gpu.h>
#include <libplacebo/dispatch.h>
#include <libplacebo/shaders.h>
#include <libplacebo/shaders/custom.h>
}

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

/**
 * Zero-latency frame generation by motion extrapolation.
 *
 * Interpolation has to hold back the newest frame until the one after it
 * arrives. This class never does that: every stream frame is shown the moment
 * it is decoded. Between two stream frames the newest frame is re-rendered,
 * warped forward along the motion measured against the frame before it.
 *
 * The GPU work happens in a libplacebo hook (RGB stage). All methods except
 * the constructor/destructor must be called from the render thread.
 */
class FrameGen
{
public:
	/// wake is called from the pacing thread whenever an extra render is due.
	explicit FrameGen(std::function<void()> wake);
	~FrameGen();

	FrameGen(const FrameGen &) = delete;
	FrameGen &operator=(const FrameGen &) = delete;

	/// multiplier: output frames per stream frame (2..4).
	void configure(bool enabled, int multiplier);
	bool enabled() const { return cfg_enabled && !broken; }

	/// A new stream frame is about to be rendered for the first time.
	void onNewFrame(uint64_t now_us, double nominal_interval_s);
	/// Call once per render, before pl_render_image. Selects the warp phase.
	void prepareRender(uint64_t now_us);
	/// Forget motion history (stream restart, resolution change, seek).
	void reset();
	/// Free GPU objects. Must run before the pl_gpu is destroyed.
	void destroyGpuResources();

	const struct pl_hook *hook() const { return &hook_def; }

	float lastPhase() const { return phase; }
	double intervalSeconds() const { return interval_s; }

private:
	static struct pl_hook_res hookCallback(void *priv, const struct pl_hook_params *params);
	struct pl_hook_res runHook(const struct pl_hook_params *params);
	bool ensureTextures(pl_gpu gpu, int w, int h);
	bool analyseNewFrame(const struct pl_hook_params *params, const float *in_off, const float *in_scale);

	void pacerLoop();
	void armPacer(double interval_s, int multiplier);
	void disarmPacer();

	struct pl_hook hook_def;

	// configuration
	bool cfg_enabled = false;
	int cfg_multiplier = 2;
	bool broken = false;

	// timing (render thread)
	uint64_t frame_serial = 0;
	uint64_t processed_serial = 0;
	uint64_t frame_time_us = 0;
	double interval_s = 0.0;
	float phase = 0.0f;

	// GPU state (render thread)
	pl_gpu last_gpu = nullptr;
	int src_w = 0;
	int src_h = 0;
	int q_w = 0, q_h = 0; // 1/4 resolution luma
	int s_w = 0, s_h = 0; // 1/16 resolution luma
	int f_w = 0, f_h = 0; // 1/8 resolution motion field
	pl_tex luma_q[2] = {nullptr, nullptr};
	pl_tex luma_s[2] = {nullptr, nullptr};
	pl_tex flow_coarse = nullptr;
	pl_tex flow_fine = nullptr;
	pl_tex flow_smooth = nullptr;
	int cur = 0;
	int history = 0;
	bool have_flow = false;

	// pacing thread
	std::function<void()> wake_cb;
	std::thread pacer;
	std::mutex pacer_mutex;
	std::condition_variable pacer_cv;
	bool pacer_stop = false;
	uint64_t pacer_generation = 0;
	bool pacer_armed = false;
	int64_t pacer_origin_ns = 0;
	int64_t pacer_step_ns = 0;
	int pacer_steps = 0;
};

#endif // CHIAKI_FRAMEGEN_H
