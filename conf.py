# https://github.com/executablebooks/MyST-Parser/blob/master/docs/conf.py

# -- Project information -----------------------------------------------------
import sys

html_static_path = ["_static"]
# html_css_files = ["custom.css"]

project = "GTSAM"
copyright = "2025, GTSAM"
author = "GTSAM" # TODO: update
version = 1.0

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.

extensions = [
    "myst_parser",
    "myst_nb"
]

html_js_files = ["https://cdnjs.cloudflare.com/ajax/libs/require.js/2.3.4/require.min.js"]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    "_build",
    "Thumbs.db", # a hidden Windows file that stores thumbnail images to speed up their display in File Explorer's thumbnail view
    ".DS_Store",
    "README.md",
]

# -- MyST settings ---------------------------------------------------

# https://myst-parser.readthedocs.io/en/latest/syntax/optional.html
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "html_image",
    "amsmath",
    "math",
    "lineno-start",
    "emphasize-lines",
    "pandoc",
    "bibliography",

    "argmin",
    "coloneqq",
    "attrs_block" # https://myst-parser.readthedocs.io/en/latest/syntax/optional.html#syntax-attributes-block
    "attrs_inline" #  used to apply syntax highlighting to inline code
]

myst_number_code_blocks = ["c", "matlab", "python",] # global default for line numbering, per lexer name