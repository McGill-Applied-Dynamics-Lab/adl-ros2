// Mermaid diagrams with fullscreen, zoom and pan.
//
// The theme renders `pre.mermaid` into a closed shadow root, which no script can reach, so the
// Mermaid fence emits `pre.mermaid-diagram` instead (see `custom_fences` in zensical.toml) and
// this script renders it: inline as usual, plus buttons to open a fullscreen view (scroll to
// zoom, drag to pan, double-click or "Reset" to fit, Esc to close) or the SVG in a new tab.
(() => {
  const MERMAID_URL = "https://unpkg.com/mermaid@11/dist/mermaid.min.js"; // same build as the theme
  let mermaidReady = null;
  let counter = 0;

  function loadMermaid() {
    if (!mermaidReady) {
      mermaidReady = new Promise((resolve, reject) => {
        if (window.mermaid) return resolve(window.mermaid);
        const script = document.createElement("script");
        script.src = MERMAID_URL;
        script.onload = () => resolve(window.mermaid);
        script.onerror = reject;
        document.head.appendChild(script);
      });
    }
    return mermaidReady;
  }

  function isDark() {
    return document.body.getAttribute("data-md-color-scheme") === "slate";
  }

  function button(label, title, onClick) {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "mermaid-zoom__button";
    b.textContent = label;
    b.title = title;
    b.addEventListener("click", onClick);
    return b;
  }

  // Fullscreen viewer: pan and zoom by rewriting the SVG viewBox
  function openViewer(svgMarkup) {
    const overlay = document.createElement("div");
    overlay.className = "mermaid-zoom__overlay";
    overlay.innerHTML = svgMarkup;
    const svg = overlay.querySelector("svg");
    svg.removeAttribute("style"); // Mermaid sets a max-width; the viewer fills the screen
    svg.setAttribute("width", "100%");
    svg.setAttribute("height", "100%");

    const base = svg.viewBox.baseVal;
    const home = { x: base.x, y: base.y, w: base.width, h: base.height };
    let view = { ...home };
    const apply = () => svg.setAttribute("viewBox", `${view.x} ${view.y} ${view.w} ${view.h}`);
    const reset = () => {
      view = { ...home };
      apply();
    };

    // Screen point -> SVG user coordinates, for zooming around the cursor
    const toSvg = (clientX, clientY) => {
      const p = svg.createSVGPoint();
      p.x = clientX;
      p.y = clientY;
      return p.matrixTransform(svg.getScreenCTM().inverse());
    };

    svg.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        const factor = Math.exp(event.deltaY * 0.0015);
        const p = toSvg(event.clientX, event.clientY);
        view = {
          x: p.x - (p.x - view.x) * factor,
          y: p.y - (p.y - view.y) * factor,
          w: view.w * factor,
          h: view.h * factor,
        };
        apply();
      },
      { passive: false },
    );

    let drag = null;
    svg.addEventListener("pointerdown", (event) => {
      drag = { start: toSvg(event.clientX, event.clientY) };
      svg.setPointerCapture(event.pointerId);
      svg.classList.add("is-dragging");
    });
    svg.addEventListener("pointermove", (event) => {
      if (!drag) return;
      const p = toSvg(event.clientX, event.clientY);
      view.x -= p.x - drag.start.x;
      view.y -= p.y - drag.start.y;
      apply();
    });
    const endDrag = () => {
      drag = null;
      svg.classList.remove("is-dragging");
    };
    svg.addEventListener("pointerup", endDrag);
    svg.addEventListener("pointercancel", endDrag);
    svg.addEventListener("dblclick", reset);

    const close = () => {
      overlay.remove();
      document.removeEventListener("keydown", onKey);
    };
    const onKey = (event) => {
      if (event.key === "Escape") close();
    };
    document.addEventListener("keydown", onKey);

    const bar = document.createElement("div");
    bar.className = "mermaid-zoom__toolbar";
    bar.append(button("Reset", "Fit the diagram (or double-click)", reset), button("Close", "Close (Esc)", close));
    const hint = document.createElement("div");
    hint.className = "mermaid-zoom__hint";
    hint.textContent = "Scroll to zoom · drag to pan · double-click to fit · Esc to close";
    overlay.append(bar, hint);
    document.body.appendChild(overlay);
    apply();
  }

  function openInTab(svgMarkup) {
    const blob = new Blob([svgMarkup], { type: "image/svg+xml" });
    window.open(URL.createObjectURL(blob), "_blank", "noopener");
  }

  async function renderAll() {
    const blocks = document.querySelectorAll("pre.mermaid-diagram");
    if (!blocks.length) return;
    const mermaid = await loadMermaid();
    mermaid.initialize({ startOnLoad: false, theme: isDark() ? "dark" : "default", securityLevel: "strict" });

    for (const pre of blocks) {
      const source = pre.textContent;
      let svgMarkup;
      try {
        ({ svg: svgMarkup } = await mermaid.render(`mermaid-zoom-${counter++}`, source));
      } catch (error) {
        console.error("mermaid-zoom: could not render diagram", error);
        continue;
      }
      const figure = document.createElement("div");
      figure.className = "mermaid-zoom";
      figure.dataset.source = source;
      const inline = document.createElement("div");
      inline.className = "mermaid-zoom__inline";
      inline.innerHTML = svgMarkup;
      inline.title = "Click to open fullscreen";
      inline.addEventListener("click", () => openViewer(svgMarkup));
      const bar = document.createElement("div");
      bar.className = "mermaid-zoom__toolbar";
      bar.append(
        button("⤢ Fullscreen", "Open fullscreen: zoom and pan", () => openViewer(svgMarkup)),
        button("↗ New tab", "Open the SVG in a new tab", () => openInTab(svgMarkup)),
      );
      figure.append(bar, inline);
      pre.replaceWith(figure);
    }
  }

  // Light/dark toggle: put the sources back and render again in the new theme
  function rerenderAll() {
    for (const figure of document.querySelectorAll(".mermaid-zoom")) {
      const pre = document.createElement("pre");
      pre.className = "mermaid-diagram";
      pre.textContent = figure.dataset.source;
      figure.replaceWith(pre);
    }
    renderAll();
  }

  function start() {
    new MutationObserver(rerenderAll).observe(document.body, {
      attributes: true,
      attributeFilter: ["data-md-color-scheme"],
    });
    // Instant navigation swaps the page without a reload: render on every page
    if (window.document$) {
      window.document$.subscribe(() => renderAll());
    } else {
      renderAll();
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", start);
  } else {
    start();
  }
})();
