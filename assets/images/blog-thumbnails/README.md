# Blog thumbnails

These small WebP images are downscaled copies of existing site artwork for the
blog listing. Original article images are unchanged. Source paths and alt text
are recorded per post in `_data/blog_thumbnails.json`; attribution remains with
the original articles and assets.

Each image fits within a 640 × 360 white canvas without cropping, stretching,
or enlarging the original. Animations use a single still frame, selected with
the optional zero-based `frame` field in the manifest. The index
displays these images at a small size and loads them lazily.

The text-only GitHub and website announcements use the existing GTSAM logo.
The text-only geometry-conventions post reuses Matias Mattamala's coordinate
frame illustration from the uncertainty series. The CustomFactor post reuses
Frank Dellaert's pose-and-landmark factor graph from “What are Factor Graphs?”;
it is an illustration, not an output of the CustomFactor examples. The CUDA
post uses the landing page's speedup chart, based on that post's benchmarks.
All other selections come from the corresponding article.

To regenerate with Ruby, ImageMagick, and librsvg's `rsvg-convert` installed,
from the repository root:

```sh
ruby scripts/resize_blog_thumbnails.rb
```

This only resizes images; it does not build Jekyll, compile GTSAM, or execute
notebooks. Add a manifest entry when adding a post, then regenerate its image.
