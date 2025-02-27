const plugin = {
  name: 'Change background and text color for headers and words',
  transforms: [
    {
      name: 'transform-colors',
      doc: 'Changes background color of specific header and all text color to purple.',
      stage: 'document',
      plugin: (_, utils) => (node) => {
        utils.selectAll('div', node).forEach((divNode) => {
          if (divNode.properties && divNode.properties.id === 'skip-to-frontmatter') {
            utils.selectAll('h1.mb-0', divNode).forEach((h1Node) => {
              h1Node['style'] = {
                backgroundColor: 'pink',  // Keep the pink background
                color: 'red',  // Change text color to red for header
              };
            });
          }
        });

        // Change color of all words in the document to purple
        utils.selectAll('text', node).forEach((textNode) => {
          textNode['style'] = {
            color: 'purple',  // Change all words' color to purple
          };
        });
      },
    },
  ],
};

export default plugin;
