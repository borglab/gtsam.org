#!/usr/bin/env ruby
# Resize existing artwork only; does not build the site or execute notebooks.
require 'json'
require 'fileutils'
require 'tempfile'

root = File.expand_path('..', __dir__)
manifest = JSON.parse(File.read(File.join(root, '_data/blog_thumbnails.json')))
posts = Dir.glob(File.join(root, '_posts/*.md')).map { |path| File.basename(path) }
missing = posts - manifest.keys
stale = manifest.keys - posts
abort "Missing thumbnails: #{missing.join(', ')}" unless missing.empty?
abort "Unknown posts: #{stale.join(', ')}" unless stale.empty?

output_dir = File.join(root, 'assets/images/blog-thumbnails')
FileUtils.mkdir_p(output_dir)
manifest.each do |post, figure|
  source = File.join(root, figure.fetch('source'))
  abort "Missing source: #{source}" unless File.file?(source)
  destination = File.join(output_dir, post.sub(/\.md\z/, '.webp'))
  Tempfile.create(['gtsam-blog-figure-', '.png']) do |raster|
    if File.extname(source).downcase == '.svg'
      # librsvg preserves paths and labels that ImageMagick's SVG reader loses.
      abort "Could not rasterize #{source}" unless system('rsvg-convert', source, '-o', raster.path)
      input = [raster.path]
    elsif figure.key?('frame')
      frame = Integer(figure.fetch('frame'))
      abort "Invalid frame for #{post}" unless frame.positive?
      # Coalesce preceding frames to preserve GIF disposal/partial updates.
      input = ["#{source}[0-#{frame}]", '-coalesce', '-delete', "0-#{frame - 1}"]
    else
      input = ["#{source}[0]"]
    end
    # Preserve the complete figure and its aspect ratio, never crop or upscale.
    success = system('magick', *input, '-auto-orient',
                     '-thumbnail', '640x360>', '-background', 'white',
                     '-alpha', 'remove', '-alpha', 'off', '-gravity', 'center',
                     '-extent', '640x360', '-strip', '-quality', '86', destination)
    abort "Could not resize #{source}" unless success
  end
end
puts "Resized #{manifest.length} blog thumbnails (640 × 360 WebP)."
