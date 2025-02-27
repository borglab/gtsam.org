WIP redesign of gtsam.org

moving to full MyST sphinx build due to Ruby Sass's deprecation

based off of https://gtsam.org/ and https://chrisholdgraf.com/

https://github.com/pydata/pydata-sphinx-theme


build:
'sphinx-autobuild . ../build/'
myst build
myst start