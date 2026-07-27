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

### Generating tutorial:

Bad flow:

1. Edit gtsam.lyx in gtsam repo
2. Export with LyxHTML
3. Add jekyll header
4. Fix all issues as axcz did :-(

Better flow would maybe use better convertor, e.g. HTML export. Manual editing of html seems non-maintainable.


### Reusing graphics

The source graphics of _"Reducing the uncertainty about the uncertainties"_ ([part 1](https://gtsam.org/2021/02/23/uncertainties-part1.html), [part 2](https://gtsam.org/2021/02/23/uncertainties-part2.html), and [part 3](https://gtsam.org/2021/02/23/uncertainties-part3.html)) are available in the file [uncertainties_graphics_main.svg](https://github.com/borglab/gtsam.org/blob/master/assets/images/uncertainties/uncertainties_graphics_main.svg) © 2021 by [Matias Mattamala](https://mmattamala.github.io), which is licensed under [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/?ref=chooser-v1) <img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt="">
