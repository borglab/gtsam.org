---
layout: page
title: Blog
permalink: /blog/
---

{% include jumbotron-other.html %}

{% for post in site.posts %}
{{ post.date | date: "%b %-d, %Y" }}:
# [{{ post.title }}]({{ post.url | prepend: site.baseurl }})
{% endfor %}
