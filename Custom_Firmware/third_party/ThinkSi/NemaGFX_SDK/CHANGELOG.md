# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [1.4.3] - 2022-10

### Fixed
- nema_font: nema_print_indexed() - x, y correspond to the top-left corner of the text

### Added
- NEMA_ERR_INVALID_CL_ALIGMENT - error when CL address is not 8 byte alligned
- nema_fill_rounded_rect_aa() - draw a filled antialized rounded rect with color
- nema_reinit() - reinitialize NemaGFX library


## [1.4.2] - 2022-06

### Added
- nema_font: nema_print_indexed() - print text using glyph indices instead of characters

### Changed
- nema_font_converter: generate glyphs for characters defined by glyph indices without unicode


## [1.4.1] - 2022-03

### Added
- nema_mat3x3_copy()


## [1.4.0] - 2021-12

### Added
- nema_math: nema_atan2() - returns arc tan in degrees
- nema_math: nema_atan2_r() - returns arc tan in radians
- nema_math: nema_cos_r() - returns cosine in radians
- nema_math: nema_sin_r() - returns sine in radians
- nema_math: nema_tan_r() - returns tangent in radians
- nema_math: nema_truncf() - returns the truncated floating point number
- nema_math: nema_fmod() - returns the floating point remainder of x/y

### Changed
- nema_math: better accuracy in nema_sqrt(), nema_cos(), nema_sin()


## [1.3.10] - 2021-10

### Added
- nema_mat3x3_rotate2()
- nema_mat3x3_rotate_pivot()
- nema_mat3x3_scale_rotate_pivot()
- PROVISIONAL: nema_draw_rounded_rect_aa()
- PROVISIONAL: nema_draw_triangle_aa()
- PROVISIONAL: nema_draw_quad_aa()
- LUT support


## [1.3.9] - 2021-04

### Fixed
- HW Rasterizer Clipping Bug - SW Workarround: enable by adding compilation
  flag -DRASTERIZER_BUG_WA
- Removed "static" qualifier from (internal) function nema_get_kern_xoffset()

### Changed
- Removed #include "nema_provisional.h" from "nema_core.h"


## [1.3.8] - 2021-02

### Added
- Support Dithering in HW (needs updated HW)

### Fixed
- Command List: fix access to freed pointer

### Changed
- PROVISIONAL: change nema_fill_triangle_strip() to nema_fill_triangle_strip_f()
- PROVISIONAL: change nema_fill_triangle_fan() to nema_fill_triangle_fan_f()
- PROVISIONAL: Support Stencil as Blender OP: NEMA_BLOP_STENCIL_XY/NEMA_BLOP_STENCIL_TXTY


## [1.3.7] - 2021-02

### Added
- PROVISIONAL: nema_fill_triangle_strip(): draw triangle strips
- PROVISIONAL: nema_fill_triangle_fan(): draw triangle fans
- platforms/baremetal_hostlink
- platforms/baremetal_tb_assist
- platforms/zc70x_linux_tb_assist

### Fixed
- nema_interpolate_tri_colors() when negative coordinates
- Ring Buffer: resolve corner case hangs when wrapping around
- baremetal_generic: nema_buffer_create() and nema_buffer_create_pool()
  recursion


## [1.3.6] - 2020-09

### Added
- Fonts: add kerning support (just add -k argument to fontConvert)


## [1.3.5] - 2020-06

### Added
- Additional Error Codes (GFX_MEMORY_INIT, DRIVER_FAILURE, MUTEX_INIT)
- Support for GP_FLAGS (available on specific HW configurations)
- int nema_cl_enough_space(int cmd_no): check if cmd_no commands can be added
  to bound CL


### Changed
- nema_font_convert tool updates

### Fixed
- tsi_malloc() fragmentation issue


## [1.3.4] - 2020-04

### Added

- Memory Footprint report in nema_font_convert utility
- When NEMA_FONT_LOAD_FROM_BIN is defined, don't instantiate font bitmaps
- nema_blit_tri_uv(): blit+fit a triangular texture area)
- nema_cl_almost_full(): returns non-zero if CL is almost full
- add Error Report mechanism. Read errors via nema_get_error() (nema_error.h)


### Changed

- Default MAX_MEM_POOLS changed from 4 to 8

### Fixed

- Bug: set correct size of subCL when extending
- Bug: nema_texture_size() - return correct size in TSC4/TSC6


## [1.3.3] - 2020-04

### Fixed

- Don't fetch (background) pixel when Hardware Blender module is enabled


## [1.3.2] - 2020-03

### Added

- Support for NEMA_A1_LE/NEMA_A2_LE/NEMA_A4_LE/NEMA_L1_LE/NEMA_L2_LE/NEMA_L4_LE
- [SW] Support for RGB24, BGR24
- Support for 64-bit address space
- Added nema_blit_rotate_pivot() (e.g. to draw a watch hand/needle)

### Changed

- Changed license in tsi_malloc.h to allow distribution



## [1.3.1] - 2019-12

### Added

- nema_ext_hold_enable/disable(), nema_ext_hold_irq_enable/disable() API calls



## [1.3] - 2019-11

### Added

- Support for 2-bit, 4-bit fonts
- Support for UTF-8 fonts
- Circular Command Lists - `void nema_cl_bind_circular(nema_cmdlist_t *cl)`
- Enable per edge HW Anti-Aliasing - `void nema_enable_aa(e0, e1, e2, e3)`
- Draw Anti-Aliased lines with width - `nema_draw_line_aa(x0, y0, x1, y1, width, color)`
- Draw Anti-Aliased Circles with width - `nema_draw_circle_aa(x, y, r, width, color)`
- Fill Anti-Aliased Circles - `nema_fill_circle_aa(x, y, r, color)`
- Blit and fit sub-rectangle to quadrilateral - `nema_blit_subrect_quad_fit()`
- Added `nema_fill_rect_f()` (floating point version of `nema_fill_rect()`)
- Added `nema_fill_quad_f()` (floating point version of `nema_fill_quad_f()`)
- Added `nema_fill_triangle_f()` (floating point version of `nema_fill_triangle_f()`)
- Support for Morton-order source textures

### Changed

- Changed struct img_obj_t (include/tsi/common/nema_utils.h)
- Moved `nema_ringbuffer_t` and `nema_rb_init()` from nema_ringbuffer.h to nema_hal.h
