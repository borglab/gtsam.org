# -- Project information -----------------------------------------------------
import sys
from source.blogpost import plugin

def setup(app):
    app.add_directive("postlist", plugin)

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

# -- ABlog ---------------------------------------------------

blog_baseurl = "https://chrisholdgraf.com"
blog_title = "Chris Holdgraf"
blog_path = "blog"
blog_post_pattern = "blogs/*/*"
blog_feed_fulltext = True
blog_feed_subtitle = "Open communities, open science, communication, and data."
fontawesome_included = True
post_redirect_refresh = 1
post_auto_image = 1
post_auto_excerpt = 2