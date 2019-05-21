---
layout: page
title: Notes
permalink: /notes/
---

{% for note in site.notes %}
# [{{ note.title }}]({{ note.url | prepend: site.baseurl }})
{% endfor %}
  
  