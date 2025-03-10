import path from 'path';

const blogPicsDirective = {
  name: 'blogpics',
  doc: 'A directive to include images from a local directory.',
  arg: {
    type: String,
    doc: 'The filename of the image to use, e.g., `my-image.jpg`',
  },
  options: {
    size: { type: String, doc: 'Size of the image, e.g., `500x300`.' },
  },

  run(data) {
    if (!data.arg) {
      throw new Error('You must provide an image filename as an argument.');
    }

    // Construct the relative path to the image
    const imagePath = `/Assets/Images/Blog/2019/09-18/${data.arg}`;

    // Parse size option
    const match = (data.options?.size ?? '').match(/^(\d+)(?:x(\d+))?$/);
    let width = null;
    let height = null;
    if (match) {
      width = match[1];
      height = match[2] || match[1]; // If only width is provided, assume square
    }

    // Return MyST-compatible image object
    return [
      {
        type: 'image',
        url: imagePath, // Relative path to the image
        width: width ? `${width}px` : undefined,
        height: height ? `${height}px` : undefined,
      },
    ];
  },
};

const plugin = { name: 'Local Image Plugin', directives: [blogPicsDirective] };

export default plugin;
