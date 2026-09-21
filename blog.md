---
layout: page
title: Blog
permalink: /blog/
---

<div class="blog-index">
{% for post in site.posts %}
  <a class="blog-card" href="{{ post.url | prepend: site.baseurl }}">
    <time datetime="{{ post.date | date_to_xmlschema }}">{{ post.date | date: "%b %-d, %Y" }}</time>
    <h2>{{ post.title }}</h2>
    <span>Read article <i aria-hidden="true">→</i></span>
  </a>
{% endfor %}
</div>
