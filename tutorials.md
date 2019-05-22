---
layout: page
title: Tutorials
permalink: /tutorials/
---

{% include jumbotron-other.html %}

{% for tutorial in site.tutorials %}
  - {{ tutorial.date | date: "%b %-d, %Y" }}
    # [{{ tutorial.title }}]({{ tutorial.url | prepend: site.baseurl }})
{% endfor %}
  
  