// Chooser page: five variant iframes stacked; each animates while on screen and
// pauses when scrolled out (postMessage to the child). The child reports its
// height, its fps and asks the parent to scroll during its walkthrough.
(function () {
  'use strict';
  const frames = new Map(); // n -> {iframe, section, status, fps, visible}
  for (const sec of document.querySelectorAll('section.variant[data-n]')) {
    const iframe = sec.querySelector('iframe');
    if (!iframe) continue;
    frames.set(sec.dataset.n, { iframe, sec, status: sec.querySelector('[data-status]'), fps: null, visible: false });
  }
  const all = location.hash.includes('all');
  const post = (f, msg) => { try { f.iframe.contentWindow.postMessage(Object.assign({ cr: 1 }, msg), '*'); } catch (e) { /* frame not ready */ } };

  window.addEventListener('message', (ev) => {
    const d = ev.data;
    if (!d || d.cr !== 1) return;
    const f = frames.get(String(d.id));
    if (!f) return;
    if (d.type === 'height' && d.h > 200) f.iframe.style.height = Math.ceil(d.h + 4) + 'px';
    if (d.type === 'ready') { f.status.textContent = 'running'; if (!f.visible && !all) post(f, { type: 'pause' }); }
    if (d.type === 'fps') { f.fps = d.fps; f.status.textContent = (f.visible || all ? 'running · ' : 'paused · ') + d.fps + ' fps'; }
    if (d.type === 'scrollTo') {
      const top = f.iframe.getBoundingClientRect().top + window.scrollY + d.y;
      window.scrollTo({ top: top - 24, behavior: matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
    }
  });

  if (!all && typeof IntersectionObserver === 'function') {
    const io = new IntersectionObserver((entries) => {
      for (const e of entries) {
        const f = [...frames.values()].find((x) => x.iframe === e.target);
        if (!f) continue;
        f.visible = e.isIntersecting;
        post(f, { type: f.visible ? 'resume' : 'pause' });
        if (f.status) f.status.textContent = (f.visible ? 'running' : 'paused') + (f.fps != null ? ' · ' + f.fps + ' fps' : '');
      }
    }, { threshold: 0, rootMargin: '0px' });
    for (const f of frames.values()) io.observe(f.iframe);
  } else {
    for (const f of frames.values()) f.visible = true;
  }

  for (const b of document.querySelectorAll('button[data-tour]')) {
    b.addEventListener('click', () => {
      const f = frames.get(b.dataset.tour);
      if (!f) return;
      f.iframe.scrollIntoView({ behavior: 'smooth', block: 'start' });
      post(f, { type: 'tour', action: 'open' });
    });
  }

  const fpsEl = document.querySelector('[data-fps]');
  setInterval(() => {
    const live = [...frames.entries()].filter(([, f]) => f.fps != null && (f.visible || all)).map(([n, f]) => n + ':' + f.fps);
    if (fpsEl) fpsEl.textContent = live.length ? 'fps ' + live.join('  ') : 'fps: no frame in view';
  }, 1000);
})();
