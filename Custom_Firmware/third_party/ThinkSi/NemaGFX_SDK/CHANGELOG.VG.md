# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [1.1.1] - 2022-10

### Fixed
 - Setting NEMA_VG_ERR_COORDS_OUT_OF_RANGE in context
 - vla in nema_vg_tsvg.c
 - opacity in predefined shapes
 - Rasterizer bug in old HW
 - Rasterizer bug when rendering polygon/polyline in PVG
 - Rasterizer bug when drawing outside fb memory in shapes

### Added
 - NEMA_VG_PAINT_MAX_GRAD_STOPS
 - NEMA_VG_ERR_INVALID_GRAD_STOPS
 - nema_vg_print_char(): print single character
 - nema_vg_init_stencil_prealloc(): user provided stencil buffer
 - nema_vg_reinit(): reinitialize NemaP regs setup for Vector Graphics


## [1.1] - 2022-07

### Added

 - Handling out of range coordinates
   nema_vg_bind_clip_coords_buf() to bind the buffer for handling coordinates out of range
   nema_vg_unbind_clip_coords_buf() to unbind the buffer for handling coordinates out of range
   nema_vg_handle_large_coords() enables handling out of range coordinates

### Fixed
 - Artifact on NemaP non zero when small triangles
 - Quality improvements


## [1.0.2] - 2022-06

### Added
 - nema_vg_draw_tsvg(): compile time #define P_FORCE_EVEN_ODD = 1 to always force use of even_odd fill rule on NemaP

### Fixed
 - Fonts: paint stroke width is not affected by font transformation
 - Random pixels showing on NemaP when fill rule is non zero


## [1.0.1] - 2022-05

### Added
 - Gradient fill paint in fonts
 - Gradient stroke paint in fonts
 - Error code NEMA_VG_ERR_NO_BOUND_FONT
 - nema_vg_init_stencil_pool() - Initialize library and allocate stencil buffer to specific memory pool
 - Added NEMA_VG_QUALITY_MAXIMUM
 - nema_vg_font.h - Added font version in font struct

 ### Changed
 - nema_vg_print() floating point arguments instead of int
 - nema_vg_string_get_bbox floating point arguments instead of int
 - nema_vg_font.c - Internal calculations using floating point
 - nema_vg_version.h - Removed get version functions and replaced by defines

### Fixed
 - Tight axis-aligned bounding box calculation
 - Cubic tesselation calculation on p
 - nema_vg_pico.cc - Rendering artifacts on NemaP using HAAS define


## [1.0]  - 2022-03 - equivalent to 0.16
## [0.16] - 2022-03

### Added
 - nema_vg_path_set_shape_with_bbox()
 - nema_vg_set_global_matrix()
 - nema_vg_grad_create()
 - nema_vg_grad_destroy()
 - nema_vg_grad_set()
 - nema_vg_paint_set_opacity()
 - nema_vg_paint_lock_tran_to_path() - Make paint follow path transformations
 - nema_vg_paint_set_grad_radial2() - Radial with rx/ry
 - NEMA_VG_STROKE
 - NEMA_VG_PAINT_COLOR

### Changed
 - nema_vg_paint_set_grad_linear() arguments
 - nema_vg_paint_set_grad_radial() arguments
 - nema_vg_paint_set_grad_conical() arguments


## [0.15] - 2022-02

### Added
 - Transformation parameter in nema_vg_draw_circle function
 - Paths/Shapes paint with LUT
 - Draw rounded rect shape function
 - Draw line shape function
 - Support for NEMA_VG_FILL_DRAW for all shape draw functions
 - Initial support of .tsvg format

### Changed
 - Draw shapes transformation parameter type
   from nema_matrix3x3_t* to nema_matrix3x3_t
 - Font rendering based on svg specification

### Fixed
 - Stencil buffer dirty area calculation
 - SubPath data start data in relative
 - Arcs with smaller radius than points distance


## [0.14] - 2022-01

### Added
 - Draw circle optimized function
 - Polygon/Polyline Segments
 - Masking Operations
 - Path stroking (Bevel joins)

### Fixed
 - Perform Transformation to Implicit Move
 - Overdraw on a single path with blending
 - Calculation of relevant control points in bezier curves
 - Calculation of smooth control points in bezier curves


## [0.13] - 2021-12

### Added
 - Elliptical arcs supporting affine transformations
 - Vector font converter utility
 - Vector fonts
 - Draw rectangle optimized function
 - Draw ellipse optimized function
 - Draw filled ring with rounded caps optimized function

### Fixed
 - Drawing SCUBIC/SQUAD when previous segment not SCUBIC/SQUAD/CUBIC/QUAD
 - Prevent path from closing implicitly in sub-path when the fill rule is draw


## [0.12] - 2021-11

### Added
 - conical gradient paint
 - radial gradient paint
 - Eliptical arcs

### Changed
 - Add _ERR in error defines

### Fixed
 - Incorrect behaviour in consecutive VLINE/HLINE
 - Incorrect behaviour when consequtive paints created but not used immediately
 - Incorrect behaviour when nema_vg_init() is called before a cl is bound
 - Incorrect behavior of nema_vg_get_error

### Known issues
 - Elliptical arcs with shear transform not showing correctly


## [0.11] - 2021-10

### Added
 - Relative Coordinates
    NEMA_VG_PRIM_LINE_REL
    NEMA_VG_PRIM_BEZIER_QUAD_REL
    NEMA_VG_PRIM_BEZIER_CUBIC_REL
    NEMA_VG_PRIM_MOVE_REL
 - Path starting from (0,0) when no move command is present
 - Implicit path closure with line when no close command is present
 - NEMA_VG_QUALITY_NON_AA
 - Smooth Segments
    NEMA_VG_PRIM_BEZIER_SQUAD
    NEMA_VG_PRIM_BEZIER_SCUBIC
    NEMA_VG_PRIM_BEZIER_SQUAD_REL
    NEMA_VG_PRIM_BEZIER_SCUBIC_REL
 - Horizontal/Vertical segments
    NEMA_VG_PRIM_HLINE
    NEMA_VG_PRIM_VLINE
    NEMA_VG_PRIM_HLINE_REL
    NEMA_VG_PRIM_VLINE_REL

### Changed
 - Rename NEMAVG_ to NEMA_VG
 - nema_vg_path_create() returns NEMA_VG_PATH_HANDLE (void*)
 - nema_vg_paint_create() returns NEMA_VG_PAINT_HANDLE (void*)
 - nema_vg_error_e enum changed to define
 - nema_vg_quality_e enum changed to define
 - nema_vg_fill_rule_e enum changed to define
 - nema_vg_primitive_e enum changed to define
 - Renamed nema_vg_paint_set_texture() to nema_vg_paint_set_tex()
 - Renamed nema_vg_paint_set_texture_matrix() to nema_vg_paint_set_tex_matrix()
 - Added argument nema_tex_mode_t sampling_mode in nema_vg_paint_set_grad_linear()
 - Remove support for explicit vertex buffer creation

### Fixed
 - NEMA_BL_ADD not performing correctly
 - Presicion in AA in non-zero filing rule
 - Correct initialization of path's bbox


## [0.10] - 2021-8

### Added
 - Blend operations
 - AA on non-zero fill rule
 - Versioning
 - nema_vg_init()
 - nema_vg_deinit()
 - Texture wrap modes (CLAMP/REPEAT/MIRROR/BORDER)
 - NEMAVG_PRIM_BEZIER_CUBIC
 - nema_vg_path_set_quality_faster()

### Changed
 - Remove nema_vg_context.h.
 - Remove nema_vg_stencil_buffer_create()
 - Remove nema_vg_stencil_buffer_destroy()
 - Remove nema_vg_set_drawing_surface()
 - Rename NEMAVG_PAINT_LINEAR_GRAD to NEMAVG_PAINT_GRAD_LINEAR
 - Rename all occurences of nemavg_* to nema_vg_*
 - Modify
    nema_vg_path_set_shape(nema_vg_path_t* path,size_t num_vertices, float* coords, uint32_t* cmds)
   to
    nema_vg_path_set_shape(nema_vg_path_t* path,int seg_size, uint8_t* seg, int data_size, nema_vg_float_t* data)
 - Rename
    nema_vg_paint_set_matrix
   to
    nema_vg_paint_set_texture_matrix
 - Add different matrixes for paint gradient and texture

### Fixed
 - Destroying paint buffer before CL was submitted.
 - Wrong paint when using the same paint consecutively
 - Not destroying an implicitely created vertex buffer when another one was explictely created
 - Wrong geometry in non-zero fill rule due to wrong fan point

### Misc
- Reduce paint buffer size


## [0.9] - 2021-06

### Changed
- Rename nema_vg_allocate_stencil_buffer to nema_vg_stencil_buffer_create
- Rename nema_vg_deallocate_stencil_buffer to nema_vg_stencil_buffer_destroy
- Refactored API - various changes

### Fixed
- Rendering artifacts - horizontal lines
- Rendering artifacts when screen resolution not a multiple of 4
- Rendering artifacts when path out of screen bounds
- Rendering artifacts due to wrong transformed bbox

### Misc
- Improve GPU performance
- Reduce CPU utilization


## [0.8] - 2021-05

### Added
- assertions
- matrix transformations (path vertices/paint vertices)
- paint/path/context
- NON-ZERO/EVEN-ODD fill rule
- linear gradient
- experimental svg-parser
- experimental ttf-parser
- examples (svg_render/matrix_transformations/basic_vg_move_examples/)
