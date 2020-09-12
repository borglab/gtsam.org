# gtsam.org

Github pages repo for https://gtsam.org/, the website for the [GTSAM](https://github.com/borglab/gtsam) sensor fusion library.

### Previewing locally

1. bundle install
2. bundle exec jekyll serve

### Writing blog posts as python notebooks:

1. Update notebook from colab by saving in notebooks folder via github integration.
2. run 
     > "jupyter nbconvert --output-dir _posts --to markdown notebooks/name.ipynb"
3. Change name to reflect date.
4. Add jekyll header.
5. Move images to assets, and change image file paths to refer to assets.

This is unsatisfactory, of course.
