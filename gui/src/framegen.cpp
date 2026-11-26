// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include "framegen.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
#endif

namespace {

int64_t steadyNowNs()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Luma of the source frame, box filtered down by 4 in each direction.
const char *const kShaderLumaFromSource =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"vec2 c = fg_in_off + uv * fg_in_scale;                                   \n"
	"vec3 a = textureLod(fg_src, c + vec2(-1.0, -1.0) * fg_px, 0.0).rgb       \n"
	"       + textureLod(fg_src, c + vec2( 1.0, -1.0) * fg_px, 0.0).rgb       \n"
	"       + textureLod(fg_src, c + vec2(-1.0,  1.0) * fg_px, 0.0).rgb       \n"
	"       + textureLod(fg_src, c + vec2( 1.0,  1.0) * fg_px, 0.0).rgb;      \n"
	"float l = dot(a * 0.25, vec3(0.2126, 0.7152, 0.0722));                   \n"
	"color = vec4(l, l, l, 1.0);                                              \n";

// Another 4x box downscale of a luma texture.
const char *const kShaderLumaDown =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"float l = textureLod(fg_src, uv + vec2(-1.0, -1.0) * fg_px, 0.0).r       \n"
	"        + textureLod(fg_src, uv + vec2( 1.0, -1.0) * fg_px, 0.0).r       \n"
	"        + textureLod(fg_src, uv + vec2(-1.0,  1.0) * fg_px, 0.0).r       \n"
	"        + textureLod(fg_src, uv + vec2( 1.0,  1.0) * fg_px, 0.0).r;      \n"
	"l *= 0.25;                                                               \n"
	"color = vec4(l, l, l, 1.0);                                              \n";

// Coarse block matching on the 1/16 luma: +-6 texels, i.e. +-96 source pixels.
// Output: xy = motion of the content (current minus previous position) in
// normalised image units, z = matching cost.
const char *const kShaderFlowCoarse =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"float best = 1.0e9;                                                      \n"
	"vec2 bestd = vec2(0.0);                                                  \n"
	"for (int dy = -6; dy <= 6; dy++) {                                       \n"
	"for (int dx = -6; dx <= 6; dx++) {                                       \n"
	"    vec2 d = vec2(float(dx), float(dy)) * fg_px;                         \n"
	"    float sad = 0.0;                                                     \n"
	"    for (int j = 0; j < 4; j++) {                                        \n"
	"    for (int i = 0; i < 4; i++) {                                        \n"
	"        vec2 o = (vec2(float(i), float(j)) - 1.5) * 2.0 * fg_px;         \n"
	"        sad += abs(textureLod(fg_cur, uv + o, 0.0).r                     \n"
	"                 - textureLod(fg_prev, uv + o + d, 0.0).r);              \n"
	"    }}                                                                   \n"
	"    sad += 0.03 * length(vec2(float(dx), float(dy)));                    \n"
	"    if (sad < best) { best = sad; bestd = d; }                           \n"
	"}}                                                                       \n"
	"color = vec4(-bestd, best, 1.0);                                         \n";

// Refinement on the 1/4 luma around the coarse vector, +-3 texels. Static
// content (HUD, minimap) is pulled to exactly zero motion. z = confidence.
const char *const kShaderFlowFine =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"vec2 base = -textureLod(fg_coarse, uv, 0.0).xy;                          \n"
	"float best = 1.0e9;                                                      \n"
	"vec2 bestd = base;                                                       \n"
	"float sad0 = 0.0;                                                        \n"
	"for (int j = 0; j < 4; j++) {                                            \n"
	"for (int i = 0; i < 4; i++) {                                            \n"
	"    vec2 o = (vec2(float(i), float(j)) - 1.5) * 2.0 * fg_px;             \n"
	"    sad0 += abs(textureLod(fg_cur, uv + o, 0.0).r                        \n"
	"              - textureLod(fg_prev, uv + o, 0.0).r);                     \n"
	"}}                                                                       \n"
	"for (int dy = -3; dy <= 3; dy++) {                                       \n"
	"for (int dx = -3; dx <= 3; dx++) {                                       \n"
	"    vec2 d = base + vec2(float(dx), float(dy)) * fg_px;                  \n"
	"    float sad = 0.0;                                                     \n"
	"    for (int j = 0; j < 4; j++) {                                        \n"
	"    for (int i = 0; i < 4; i++) {                                        \n"
	"        vec2 o = (vec2(float(i), float(j)) - 1.5) * 2.0 * fg_px;         \n"
	"        sad += abs(textureLod(fg_cur, uv + o, 0.0).r                     \n"
	"                 - textureLod(fg_prev, uv + o + d, 0.0).r);              \n"
	"    }}                                                                   \n"
	"    sad += 0.004 * length(vec2(float(dx), float(dy)));                   \n"
	"    if (sad < best) { best = sad; bestd = d; }                           \n"
	"}}                                                                       \n"
	"if (sad0 <= best + 0.04) { best = sad0; bestd = vec2(0.0); }             \n"
	"float conf = clamp(1.0 - (best / 16.0 - 0.015) / 0.06, 0.0, 1.0);        \n"
	"color = vec4(-bestd, conf, 1.0);                                         \n";

// Confidence weighted 3x3 smoothing of the motion field.
const char *const kShaderFlowSmooth =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"vec2 acc = vec2(0.0);                                                    \n"
	"float wsum = 0.0;                                                        \n"
	"float csum = 0.0;                                                        \n"
	"for (int j = -1; j <= 1; j++) {                                          \n"
	"for (int i = -1; i <= 1; i++) {                                          \n"
	"    vec4 f = textureLod(fg_flow, uv + vec2(float(i), float(j)) * fg_px, 0.0); \n"
	"    float w = (f.z + 0.05) * ((i == 0 && j == 0) ? 4.0 : 1.0);           \n"
	"    acc += f.xy * w;                                                     \n"
	"    wsum += w;                                                           \n"
	"    csum += f.z * ((i == 0 && j == 0) ? 4.0 : 1.0);                      \n"
	"}}                                                                       \n"
	"color = vec4(acc / wsum, csum / 12.0, 1.0);                              \n";

// Warp the newest frame forward by fg_phase of a frame interval. Where the
// motion is uncertain the vector shrinks to zero and the real pixel is shown.
const char *const kShaderWarp =
	"vec2 uv = gl_FragCoord.xy / fg_tsize;                                   \n"
	"vec4 f0 = textureLod(fg_flow, uv, 0.0);                                  \n"
	"vec2 p1 = uv - fg_phase * f0.xy * f0.z;                                  \n"
	"vec4 f1 = textureLod(fg_flow, p1, 0.0);                                  \n"
	"vec2 src = clamp(uv - fg_phase * f1.xy * f1.z, vec2(0.0), vec2(1.0));    \n"
	"color = textureLod(fg_src, fg_in_off + src * fg_in_scale, 0.0);          \n";

struct PassTex
{
	const char *name;
	pl_tex tex;
};

struct PassVar
{
	const char *name;
	const float *data;
	int components; // 1 or 2
};

bool runPass(pl_dispatch dp, pl_tex target, const char *description, const char *body,
		const PassTex *textures, int num_textures, const PassVar *vars, int num_vars)
{
	struct pl_shader_desc descs[4] = {};
	struct pl_shader_var svars[6] = {};
	if (num_textures > 4 || num_vars > 5)
		return false;

	for (int i = 0; i < num_textures; i++) {
		descs[i].desc.name = textures[i].name;
		descs[i].desc.type = PL_DESC_SAMPLED_TEX;
		descs[i].binding.object = textures[i].tex;
		descs[i].binding.address_mode = PL_TEX_ADDRESS_CLAMP;
		descs[i].binding.sample_mode = PL_TEX_SAMPLE_LINEAR;
	}

	const float tsize[2] = {
		static_cast<float>(target->params.w),
		static_cast<float>(target->params.h),
	};
	svars[0].var = pl_var_vec2("fg_tsize");
	svars[0].data = tsize;
	svars[0].dynamic = false;
	for (int i = 0; i < num_vars; i++) {
		svars[i + 1].var = vars[i].components == 2 ? pl_var_vec2(vars[i].name) : pl_var_float(vars[i].name);
		svars[i + 1].data = vars[i].data;
		svars[i + 1].dynamic = true;
	}

	pl_shader sh = pl_dispatch_begin(dp);
	if (!sh)
		return false;

	struct pl_custom_shader cs = {};
	cs.description = description;
	cs.body = body;
	cs.input = PL_SHADER_SIG_NONE;
	cs.output = PL_SHADER_SIG_COLOR;
	cs.descriptors = descs;
	cs.num_descriptors = num_textures;
	cs.variables = svars;
	cs.num_variables = num_vars + 1;
	if (!pl_shader_custom(sh, &cs)) {
		pl_dispatch_abort(dp, &sh);
		return false;
	}

	struct pl_dispatch_params params = {};
	params.shader = &sh;
	params.target = target;
	return pl_dispatch_finish(dp, &params);
}

bool recreateTex(pl_gpu gpu, pl_fmt fmt, pl_tex *tex, int w, int h)
{
	struct pl_tex_params params = {};
	params.w = w;
	params.h = h;
	params.format = fmt;
	params.sampleable = true;
	params.renderable = true;
	return pl_tex_recreate(gpu, tex, &params);
}

} // namespace

FrameGen::FrameGen(std::function<void()> wake)
	: wake_cb(std::move(wake))
{
	hook_def = {};
	hook_def.stages = PL_HOOK_RGB;
	hook_def.input = PL_HOOK_SIG_TEX;
	hook_def.priv = this;
	hook_def.hook = &FrameGen::hookCallback;
	hook_def.signature = 0x6672616d6567656eULL; // "framegen"
	pacer = std::thread([this]() { pacerLoop(); });
}

FrameGen::~FrameGen()
{
	{
		std::lock_guard<std::mutex> lock(pacer_mutex);
		pacer_stop = true;
		pacer_generation++;
	}
	pacer_cv.notify_all();
	if (pacer.joinable())
		pacer.join();
}

void FrameGen::configure(bool enabled, int multiplier)
{
	multiplier = std::max(2, std::min(4, multiplier));
	if (enabled == cfg_enabled && multiplier == cfg_multiplier)
		return;
	const bool was_enabled = cfg_enabled;
	cfg_enabled = enabled;
	cfg_multiplier = multiplier;
	if (!enabled) {
		disarmPacer();
		phase = 0.0f;
	}
	if (enabled && !was_enabled) {
		broken = false;
		reset();
	}
}

void FrameGen::reset()
{
	history = 0;
	have_flow = false;
	frame_time_us = 0;
	interval_s = 0.0;
	phase = 0.0f;
	disarmPacer();
}

void FrameGen::onNewFrame(uint64_t now_us, double nominal_interval_s)
{
	if (!enabled())
		return;

	if (nominal_interval_s < 0.004 || nominal_interval_s > 0.1)
		nominal_interval_s = 1.0 / 30.0;

	if (frame_time_us && now_us > frame_time_us) {
		const double delta_s = static_cast<double>(now_us - frame_time_us) / 1000000.0;
		if (interval_s <= 0.0)
			interval_s = nominal_interval_s;
		// Ignore stalls and bursts; follow the real cadence otherwise.
		if (delta_s > interval_s * 0.4 && delta_s < interval_s * 2.5)
			interval_s = interval_s * 0.9 + delta_s * 0.1;
		else if (delta_s >= interval_s * 2.5)
			have_flow = false; // motion measured across a gap is not trustworthy
	} else {
		interval_s = nominal_interval_s;
	}
	interval_s = std::max(0.004, std::min(0.1, interval_s));

	frame_time_us = now_us;
	frame_serial++;
	phase = 0.0f;
	armPacer(interval_s, cfg_multiplier);
}

void FrameGen::prepareRender(uint64_t now_us)
{
	if (!enabled() || !frame_time_us || interval_s <= 0.0) {
		phase = 0.0f;
		return;
	}
	if (frame_serial != processed_serial) {
		// First render of this frame: show it untouched.
		phase = 0.0f;
		return;
	}
	const double elapsed_s = now_us > frame_time_us
		? static_cast<double>(now_us - frame_time_us) / 1000000.0
		: 0.0;
	// Snap to the pacing grid so that every generated frame sits on an even
	// fraction of the interval, and never extrapolate past the next frame.
	const double steps = std::floor(elapsed_s / interval_s * cfg_multiplier + 0.5);
	const double max_steps = static_cast<double>(cfg_multiplier - 1);
	phase = static_cast<float>(std::max(0.0, std::min(max_steps, steps)) / cfg_multiplier);
}

void FrameGen::destroyGpuResources()
{
	disarmPacer();
	if (last_gpu) {
		for (int i = 0; i < 2; i++) {
			pl_tex_destroy(last_gpu, &luma_q[i]);
			pl_tex_destroy(last_gpu, &luma_s[i]);
		}
		pl_tex_destroy(last_gpu, &flow_coarse);
		pl_tex_destroy(last_gpu, &flow_fine);
		pl_tex_destroy(last_gpu, &flow_smooth);
	}
	last_gpu = nullptr;
	src_w = src_h = 0;
	history = 0;
	have_flow = false;
}

bool FrameGen::ensureTextures(pl_gpu gpu, int w, int h)
{
	if (gpu == last_gpu && w == src_w && h == src_h && flow_smooth)
		return true;

	if (last_gpu && last_gpu != gpu)
		destroyGpuResources();

	const pl_fmt fmt = pl_find_fmt(gpu, PL_FMT_FLOAT, 4, 16, 0,
		static_cast<enum pl_fmt_caps>(PL_FMT_CAP_SAMPLEABLE | PL_FMT_CAP_RENDERABLE | PL_FMT_CAP_LINEAR));
	if (!fmt)
		return false;

	last_gpu = gpu;
	q_w = std::max(16, w / 4);
	q_h = std::max(16, h / 4);
	s_w = std::max(8, q_w / 4);
	s_h = std::max(8, q_h / 4);
	f_w = std::max(8, q_w / 2);
	f_h = std::max(8, q_h / 2);

	bool ok = true;
	for (int i = 0; i < 2; i++) {
		ok = ok && recreateTex(gpu, fmt, &luma_q[i], q_w, q_h);
		ok = ok && recreateTex(gpu, fmt, &luma_s[i], s_w, s_h);
	}
	ok = ok && recreateTex(gpu, fmt, &flow_coarse, s_w, s_h);
	ok = ok && recreateTex(gpu, fmt, &flow_fine, f_w, f_h);
	ok = ok && recreateTex(gpu, fmt, &flow_smooth, f_w, f_h);
	if (!ok) {
		destroyGpuResources();
		return false;
	}

	src_w = w;
	src_h = h;
	history = 0;
	have_flow = false;
	return true;
}

bool FrameGen::analyseNewFrame(const struct pl_hook_params *params, const float *in_off, const float *in_scale)
{
	pl_dispatch dp = params->dispatch;
	cur = 1 - cur;
	const int prev = 1 - cur;

	const float src_px[2] = {
		1.0f / static_cast<float>(params->tex->params.w),
		1.0f / static_cast<float>(params->tex->params.h),
	};
	const float q_px[2] = {1.0f / q_w, 1.0f / q_h};
	const float s_px[2] = {1.0f / s_w, 1.0f / s_h};
	const float f_px[2] = {1.0f / f_w, 1.0f / f_h};

	{
		const PassTex tex[] = {{"fg_src", params->tex}};
		const PassVar vars[] = {{"fg_in_off", in_off, 2}, {"fg_in_scale", in_scale, 2}, {"fg_px", src_px, 2}};
		if (!runPass(dp, luma_q[cur], "framegen luma 1/4", kShaderLumaFromSource, tex, 1, vars, 3))
			return false;
	}
	{
		const PassTex tex[] = {{"fg_src", luma_q[cur]}};
		const PassVar vars[] = {{"fg_px", q_px, 2}};
		if (!runPass(dp, luma_s[cur], "framegen luma 1/16", kShaderLumaDown, tex, 1, vars, 1))
			return false;
	}

	if (history < 1) {
		history++;
		have_flow = false;
		return true;
	}

	{
		const PassTex tex[] = {{"fg_cur", luma_s[cur]}, {"fg_prev", luma_s[prev]}};
		const PassVar vars[] = {{"fg_px", s_px, 2}};
		if (!runPass(dp, flow_coarse, "framegen motion coarse", kShaderFlowCoarse, tex, 2, vars, 1))
			return false;
	}
	{
		const PassTex tex[] = {{"fg_cur", luma_q[cur]}, {"fg_prev", luma_q[prev]}, {"fg_coarse", flow_coarse}};
		const PassVar vars[] = {{"fg_px", q_px, 2}};
		if (!runPass(dp, flow_fine, "framegen motion fine", kShaderFlowFine, tex, 3, vars, 1))
			return false;
	}
	{
		const PassTex tex[] = {{"fg_flow", flow_fine}};
		const PassVar vars[] = {{"fg_px", f_px, 2}};
		if (!runPass(dp, flow_smooth, "framegen motion smooth", kShaderFlowSmooth, tex, 1, vars, 1))
			return false;
	}

	have_flow = true;
	return true;
}

struct pl_hook_res FrameGen::hookCallback(void *priv, const struct pl_hook_params *params)
{
	return static_cast<FrameGen *>(priv)->runHook(params);
}

struct pl_hook_res FrameGen::runHook(const struct pl_hook_params *params)
{
	struct pl_hook_res res = {};
	res.output = PL_HOOK_SIG_NONE;

	if (!enabled() || !params->tex || !params->dispatch || !params->gpu)
		return res;

	const float rect_w = params->rect.x1 - params->rect.x0;
	const float rect_h = params->rect.y1 - params->rect.y0;
	const int w = static_cast<int>(std::lround(std::fabs(rect_w)));
	const int h = static_cast<int>(std::lround(std::fabs(rect_h)));
	if (w < 64 || h < 64)
		return res;

	if (!ensureTextures(params->gpu, w, h)) {
		broken = true;
		disarmPacer();
		return res;
	}

	const float tex_w = static_cast<float>(params->tex->params.w);
	const float tex_h = static_cast<float>(params->tex->params.h);
	const float in_off[2] = {params->rect.x0 / tex_w, params->rect.y0 / tex_h};
	const float in_scale[2] = {rect_w / tex_w, rect_h / tex_h};

	if (frame_serial != processed_serial) {
		processed_serial = frame_serial;
		if (!analyseNewFrame(params, in_off, in_scale)) {
			broken = true;
			disarmPacer();
			return res;
		}
	}

	if (!have_flow || phase <= 0.001f)
		return res;

	pl_tex out = params->get_tex ? params->get_tex(params->priv, w, h) : nullptr;
	if (!out)
		return res;

	const PassTex tex[] = {{"fg_src", params->tex}, {"fg_flow", flow_smooth}};
	const PassVar vars[] = {{"fg_in_off", in_off, 2}, {"fg_in_scale", in_scale, 2}, {"fg_phase", &phase, 1}};
	if (!runPass(params->dispatch, out, "framegen warp", kShaderWarp, tex, 2, vars, 3)) {
		broken = true;
		disarmPacer();
		return res;
	}

	res.output = PL_HOOK_SIG_TEX;
	res.tex = out;
	res.repr = params->repr;
	res.color = params->color;
	res.components = params->components;
	res.rect.x0 = 0.0f;
	res.rect.y0 = 0.0f;
	res.rect.x1 = static_cast<float>(w);
	res.rect.y1 = static_cast<float>(h);
	return res;
}

void FrameGen::armPacer(double interval, int multiplier)
{
	{
		std::lock_guard<std::mutex> lock(pacer_mutex);
		pacer_generation++;
		pacer_armed = true;
		pacer_origin_ns = steadyNowNs();
		pacer_step_ns = static_cast<int64_t>(interval * 1.0e9 / multiplier);
		pacer_steps = multiplier - 1;
	}
	pacer_cv.notify_all();
}

void FrameGen::disarmPacer()
{
	{
		std::lock_guard<std::mutex> lock(pacer_mutex);
		if (!pacer_armed)
			return;
		pacer_generation++;
		pacer_armed = false;
	}
	pacer_cv.notify_all();
}

void FrameGen::pacerLoop()
{
#ifdef _WIN32
	HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr,
		CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
#endif
	std::unique_lock<std::mutex> lock(pacer_mutex);
	while (!pacer_stop) {
		if (!pacer_armed) {
			pacer_cv.wait(lock);
			continue;
		}
		const uint64_t generation = pacer_generation;
		const int64_t origin_ns = pacer_origin_ns;
		const int64_t step_ns = pacer_step_ns;
		const int steps = pacer_steps;
		lock.unlock();

		bool superseded = false;
		for (int k = 1; k <= steps && !superseded; k++) {
			const int64_t deadline_ns = origin_ns + step_ns * k;
			for (;;) {
				const int64_t remaining_ns = deadline_ns - steadyNowNs();
				if (remaining_ns <= 0)
					break;
				bool slept = false;
#ifdef _WIN32
				if (timer && remaining_ns > 1500000) {
					LARGE_INTEGER due;
					due.QuadPart = -((remaining_ns - 1000000) / 100);
					if (SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE)) {
						WaitForSingleObject(timer, 50);
						slept = true;
					}
				}
#else
				if (remaining_ns > 300000) {
					std::this_thread::sleep_for(std::chrono::nanoseconds(remaining_ns - 200000));
					slept = true;
				}
#endif
				if (!slept)
					std::this_thread::yield();
				lock.lock();
				superseded = pacer_stop || pacer_generation != generation;
				lock.unlock();
				if (superseded)
					break;
			}
			if (superseded)
				break;
			lock.lock();
			superseded = pacer_stop || pacer_generation != generation;
			lock.unlock();
			if (!superseded && wake_cb)
				wake_cb();
		}

		lock.lock();
		if (!pacer_stop && pacer_generation == generation)
			pacer_armed = false;
	}
#ifdef _WIN32
	if (timer)
		CloseHandle(timer);
#endif
}
