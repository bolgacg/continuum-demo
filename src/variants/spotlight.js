// Spotlight walkthrough (DEMO-STANDARDS section 6): dims the page, cuts a
// window around one element, and puts a card beside it with Back, Next, Close
// and progress dots. The dim layer takes no pointer events, so every control
// keeps working while the tour is open. Placement is computed from document
// coordinates, then the page scrolls; nothing waits on a timer.
// Ported from grid-health-demo/app.js with the embedded (iframe) case added.
(function (CR) {
  'use strict';

  function createTour(steps, opts) {
    const o = opts || {};
    const doc = document;
    const root = doc.createElement('div');
    root.className = 'cr-tour';
    root.innerHTML = '<div class="cr-tour-hl"></div><div class="cr-tour-card" role="dialog" aria-live="polite"></div>';
    doc.body.appendChild(root);
    const hl = root.firstChild, card = root.lastChild;
    let idx = 0, open = false;
    const reduce = () => matchMedia('(prefers-reduced-motion: reduce)').matches;
    const embedded = () => window !== window.top;

    function scrollTo(y) {
      if (embedded() && window.CR_EMBED) { window.CR_EMBED.scrollTo(y); return; }
      // long jumps go instantly: a smooth scroll over thousands of pixels reads as a lost cursor
      const far = Math.abs(window.scrollY - y) > 2500;
      window.scrollTo({ top: y, behavior: reduce() || far ? 'auto' : 'smooth' });
    }
    function place() {
      const st = steps[idx];
      const el = st && doc.querySelector(st.target);
      if (!el) { if (idx < steps.length - 1) { idx++; place(); } else close(); return; }
      if (st.onEnter) { try { st.onEnter(); } catch (e) { /* the tour must survive an act's error */ } }
      const r = el.getBoundingClientRect(), sx = window.scrollX, sy = window.scrollY;
      const top = r.top + sy, left = r.left + sx;
      root.style.height = doc.documentElement.scrollHeight + 'px';
      hl.style.left = (left - 8) + 'px'; hl.style.top = (top - 8) + 'px';
      hl.style.width = (r.width + 16) + 'px'; hl.style.height = (r.height + 16) + 'px';
      const dots = steps.map((_, i) => '<i class="' + (i === idx ? 'on' : '') + '"></i>').join('');
      const body = typeof st.body === 'function' ? st.body() : st.body;
      card.innerHTML = '<div class="tk">' + (st.title || '') + ' · ' + (idx + 1) + ' of ' + steps.length + '</div><p>' + body + '</p>' +
        '<div class="cr-tour-nav"><div class="dots">' + dots + '</div>' +
        (idx > 0 ? '<button class="cr-tour-btn" data-t="back">Back</button>' : '') +
        '<button class="cr-tour-btn" data-t="close">Close</button>' +
        '<button class="cr-tour-btn primary" data-t="next">' + (idx < steps.length - 1 ? 'Next' : 'Done') + '</button></div>';
      const cw = Math.min(400, window.innerWidth - 32);
      const narrow = window.innerWidth < 700;
      let cx = left + r.width + 18, cy = top;
      if (narrow || st.place === 'below' || cx + cw > sx + window.innerWidth - 16) { cx = Math.max(16 + sx, Math.min(left, sx + window.innerWidth - cw - 16)); cy = top + r.height + 14; }
      card.style.left = cx + 'px'; card.style.top = cy + 'px';
      card.querySelector('[data-t="next"]').onclick = next;
      card.querySelector('[data-t="close"]').onclick = close;
      const b = card.querySelector('[data-t="back"]'); if (b) b.onclick = back;
      // scroll so the highlighted element sits above its card and inside the viewport
      const want = top - Math.max(24, (window.innerHeight - r.height - 260) / 2);
      scrollTo(Math.max(0, want));
    }
    function next() { if (idx >= steps.length - 1) { close(); return; } idx++; place(); }
    function back() { idx = Math.max(0, idx - 1); place(); }
    function goto(i) { idx = Math.max(0, Math.min(steps.length - 1, i)); if (!open) openAt(idx); else place(); }
    function openAt(i) { idx = i || 0; open = true; root.classList.add('on'); place(); }
    function close() { open = false; root.classList.remove('on'); try { if (o.storageKey) localStorage.setItem(o.storageKey, 'done'); } catch (e) { /* storage may be unavailable */ } }
    function onKey(ev) {
      if (!open) return;
      if (ev.key === 'Escape') close();
      else if (ev.key === 'ArrowRight') next();
      else if (ev.key === 'ArrowLeft') back();
    }
    doc.addEventListener('keydown', onKey);
    window.addEventListener('resize', () => { if (open) place(); });
    if (o.button) { const b = typeof o.button === 'string' ? doc.querySelector(o.button) : o.button; if (b) b.addEventListener('click', () => openAt(0)); }
    let seen = null;
    try { seen = o.storageKey ? localStorage.getItem(o.storageKey) : null; } catch (e) { seen = null; }
    if (o.autoOpenOnce && !seen && !embedded()) setTimeout(() => openAt(0), 700);

    const api = { open: openAt, close, next, back, goto, isOpen: () => open, place, steps,
      destroy() { doc.removeEventListener('keydown', onKey); root.remove(); } };
    return api;
  }

  CR.spotlight = { createTour };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
