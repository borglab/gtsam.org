---
layout: page
title: Notes
permalink: /notes/
---

{% include jumbotron-other.html %}

{% for note in site.notes %}
# [{{ note.title }}]({{ note.url | prepend: site.baseurl }})
{% endfor %}
  
  