"""Crops images from MS PowerPoint png export to consistent size
Note: please export png's at width 3600
"""
from PIL import Image
from PIL.ImageOps import invert
from os import chdir, listdir

def find_bounds(im_names):
    print('finding max bounds in {}'.format(im_names))
    max_bounds = [0,]*4 # left, upper, right, lower
    min_bounds = [999999,]*4
    for im_name in im_names:
        with Image.open(im_name) as im:
            bounds = invert(im).getbbox()
            max_bounds = [max(bounds[i], max_bounds[i]) for i in range(4)]
            min_bounds = [min(bounds[i], min_bounds[i]) for i in range(4)]
    return (min(max_bounds[0], min_bounds[0]),
            min(max_bounds[1], min_bounds[1]),
            max(max_bounds[2], min_bounds[2]),
            max(max_bounds[3], min_bounds[3]))
def crop(im_name, bounds):
    print('cropping {}\tto size {}'.format(im_name, bounds))
    with Image.open(im_name) as im:
        im = im.crop(bounds)
        im.save('cropped_'+im_name)

def main():
    chdir('Elimination/')
    im_names = listdir('./')
    im_names.sort()
    im_names = [im for im in im_names
                    if im.endswith('.png') and ('cropped_' not in im)]
    bounds = find_bounds(im_names)
    for im_name in im_names:
        crop(im_name, bounds)

if __name__ == '__main__':
    main()