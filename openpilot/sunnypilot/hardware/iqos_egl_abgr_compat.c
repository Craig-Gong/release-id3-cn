// IQ.OS 4.9 Adreno EGL exposes ARGB8888 (AR24) window configs, not ABGR8888
// (AB24). Official comma raylib only accepts AB24, so InitWindow fails and
// the boot logo never yields the screen. Remap the visual id and GBM format.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stddef.h>
#include <stdint.h>

#define EGL_NATIVE_VISUAL_ID 0x302E
#define GBM_FORMAT_ABGR8888 0x34324241u
#define GBM_FORMAT_ARGB8888 0x34325241u

typedef void *EGLDisplay;
typedef void *EGLConfig;
typedef int32_t EGLint;
typedef unsigned int EGLBoolean;

static void *libegl(void) {
  static void *h = NULL;
  if (h == NULL) {
    h = dlopen("libEGL.so.1", RTLD_NOW | RTLD_NOLOAD);
    if (h == NULL) {
      h = dlopen("libEGL.so.1", RTLD_NOW);
    }
  }
  return h;
}

static void *libgbm(void) {
  static void *h = NULL;
  if (h == NULL) {
    h = dlopen("libgbm.so.1", RTLD_NOW | RTLD_NOLOAD);
    if (h == NULL) {
      h = dlopen("libgbm.so.1", RTLD_NOW);
    }
  }
  return h;
}

EGLBoolean eglGetConfigAttrib(EGLDisplay dpy, EGLConfig config, EGLint attribute, EGLint *value) {
  static EGLBoolean (*real)(EGLDisplay, EGLConfig, EGLint, EGLint *) = NULL;
  if (real == NULL) {
    real = dlsym(libegl(), "eglGetConfigAttrib");
  }
  EGLBoolean ok = real(dpy, config, attribute, value);
  if (ok && attribute == EGL_NATIVE_VISUAL_ID && value != NULL && (uint32_t)*value == GBM_FORMAT_ARGB8888) {
    *value = (EGLint)GBM_FORMAT_ABGR8888;
  }
  return ok;
}

struct gbm_device;
struct gbm_surface;

struct gbm_surface *gbm_surface_create(struct gbm_device *gbm, uint32_t width, uint32_t height,
                                       uint32_t format, uint32_t flags) {
  static struct gbm_surface *(*real)(struct gbm_device *, uint32_t, uint32_t, uint32_t, uint32_t) = NULL;
  if (real == NULL) {
    real = dlsym(libgbm(), "gbm_surface_create");
  }
  if (format == GBM_FORMAT_ABGR8888) {
    format = GBM_FORMAT_ARGB8888;
  }
  return real(gbm, width, height, format, flags);
}
