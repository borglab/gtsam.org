import path from 'path';

const localpicsDirective = {
  name: 'localpics',
  doc: 'A directive to include images from a local directory.',
  arg: {
    type: String,
    doc: 'The filename of the image to use, e.g., `my-image.jpg`',
  },
  options: {
    size: { type: String, doc: 'Size of the image, e.g., `500x300`.' },
    scale: { type: Number, doc: 'Scale of the image as a percentage, e.g., `50` for 50%. Default is 85%.' },
  },

  run(data) {
    if (!data.arg) {
      throw new Error('You must provide an image filename as an argument.');
    }

    // Construct relative path to the image
    const imagePath = `_static/${data.arg}`;

    // Preserve default image dimensions unless size is explicitly set
    let width = undefined;
    let height = undefined;
    let scale = data.options?.scale ?? 85; // Default scale to 85%

    // If a size is provided, parse it
    if (data.options?.size) {
      const match = data.options.size.match(/^(\d+)(?:x(\d+))?$/);
      if (match) {
        width = `${(parseInt(match[1]) * scale) / 100}px`;
        height = match[2] ? `${(parseInt(match[2]) * scale) / 100}px` : "auto";
      }
    }

    console.log("Image Path: ", imagePath); // Debugging output
    console.log("Scale: ", scale, "%");

    const imageElement = {
      type: 'image',
      url: imagePath,
      width,
      height,
    };

    return [imageElement];
  },
};

const plugin = { name: 'Local Image Plugin', directives: [localpicsDirective] };

export default plugin;
