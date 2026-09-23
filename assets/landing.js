/* Navigation feedback only: never hide content or take over scrolling. */
(function () {
  'use strict';
  var links = Array.from(document.querySelectorAll('.release-chapters a'));
  var chapters = links.map(function (link) {
    return { link: link, section: document.getElementById(link.hash.slice(1)) };
  }).filter(function (chapter) { return chapter.section; });
  var pending = false;

  function update() {
    pending = false;
    chapters.forEach(function (chapter) {
      var bounds = chapter.section.getBoundingClientRect();
      if (bounds.top <= 110 && bounds.bottom > 110) {
        chapter.link.setAttribute('aria-current', 'location');
      } else {
        chapter.link.removeAttribute('aria-current');
      }
    });
  }

  function schedule() {
    if (!pending) {
      pending = true;
      window.requestAnimationFrame(update);
    }
  }

  window.addEventListener('scroll', schedule, { passive: true });
  window.addEventListener('resize', schedule);
  window.addEventListener('load', schedule);
  update();
}());
