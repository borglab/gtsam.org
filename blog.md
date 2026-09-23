---
layout: page
title: Blog
permalink: /blog/
---

<div class="blog-index">
{% for post in site.posts %}
  {% assign post_file = post.path | split: '/' | last %}
  {% assign thumbnail = site.data.blog_thumbnails[post_file] %}
  <a class="blog-card" href="{{ post.url | prepend: site.baseurl }}">
    {% if thumbnail %}
    {% assign thumbnail_url = post_file | replace: '.md', '.webp' | prepend: '/assets/images/blog-thumbnails/' %}
    <img class="blog-card-thumbnail" src="{{ thumbnail_url | relative_url }}" alt="{{ thumbnail.alt | escape }}" width="640" height="360" loading="lazy" decoding="async">
    {% endif %}
    <time datetime="{{ post.date | date_to_xmlschema }}">{{ post.date | date: "%b %-d, %Y" }}</time>
    <h2>{{ post.title }}</h2>
    <span>Read article <i aria-hidden="true">→</i></span>
  </a>
{% endfor %}
</div>
