# -- Project information -----------------------------------------------------
import sys

html_static_path = ["_static"]
html_css_files = ["custom.css"]

project = "GTSAM"
copyright = "2025, GTSAM"
author = "GTSAM"

extensions = [
    "myst_nb",
    "sphinx_design",
    "sphinx_copybutton",
    "sphinx_examples",
    "sphinxext.opengraph",
    "sphinxext.rediraffe",

    "sphinx-subfigure", # https://sphinx-subfigure.readthedocs.io/en/latest/
    "sphinxcontrib.subfigure",
]

templates_path = ["_templates"]
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "*import_posts*",
    "**/pandoc_ipynb/inputs/*",
    ".nox/*",
    "README.md",
    "**/.ipynb_checkpoints/*",
]

# -- MyST and MyST-NB ---------------------------------------------------

# MyST
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "html_image",
    "amsmath",
    "math",
    "lineno-start",
    "emphasize-lines",
    "pandoc",
    "bibliography"
]

myst_plugins = [
    "Scripts.emphasis"  # Replace with the actual path to your plugin
]