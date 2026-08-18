/* @ds-bundle: {"namespace":"UOSSlideDS","components":[{"name":"AccentTab","sourcePath":"components/general/AccentTab/AccentTab.jsx"},{"name":"BulletList","sourcePath":"components/general/BulletList/BulletList.jsx"},{"name":"ChapterSlide","sourcePath":"components/general/ChapterSlide/ChapterSlide.jsx"},{"name":"ContentSlide","sourcePath":"components/general/ContentSlide/ContentSlide.jsx"},{"name":"DataTable","sourcePath":"components/general/DataTable/DataTable.jsx"},{"name":"DiagonalPanel","sourcePath":"components/general/DiagonalPanel/DiagonalPanel.jsx"},{"name":"FigureFrame","sourcePath":"components/general/FigureFrame/FigureFrame.jsx"},{"name":"Slide","sourcePath":"components/general/Slide/Slide.jsx"},{"name":"StatCard","sourcePath":"components/general/StatCard/StatCard.jsx"},{"name":"TitleSlide","sourcePath":"components/general/TitleSlide/TitleSlide.jsx"},{"name":"TocSlide","sourcePath":"components/general/TocSlide/TocSlide.jsx"},{"name":"UosLogo","sourcePath":"components/general/UosLogo/UosLogo.jsx"}],"sourceHashes":{"components/general/AccentTab/AccentTab.jsx":"8abef7bf271f","components/general/AccentTab/AccentTab.d.ts":"9248e0596ff8","components/general/AccentTab/AccentTab.prompt.md":"c9967b66efa0","components/general/BulletList/BulletList.jsx":"da408de7b49a","components/general/BulletList/BulletList.d.ts":"d95c0d9db1a6","components/general/BulletList/BulletList.prompt.md":"7e0ee2c269f3","components/general/ChapterSlide/ChapterSlide.jsx":"df1d4f2dd241","components/general/ChapterSlide/ChapterSlide.d.ts":"aebf27ff94dc","components/general/ChapterSlide/ChapterSlide.prompt.md":"af730c876d97","components/general/ContentSlide/ContentSlide.jsx":"58132d3b7fb2","components/general/ContentSlide/ContentSlide.d.ts":"86d0ce31eff5","components/general/ContentSlide/ContentSlide.prompt.md":"c6ceff3f9541","components/general/DataTable/DataTable.jsx":"53fb3dd2e3fa","components/general/DataTable/DataTable.d.ts":"cf4e211d68e0","components/general/DataTable/DataTable.prompt.md":"778f6bfbe757","components/general/DiagonalPanel/DiagonalPanel.jsx":"44fc98ef55de","components/general/DiagonalPanel/DiagonalPanel.d.ts":"12b1c2b3251d","components/general/DiagonalPanel/DiagonalPanel.prompt.md":"43723816be77","components/general/FigureFrame/FigureFrame.jsx":"8b0be8dcb358","components/general/FigureFrame/FigureFrame.d.ts":"01513d1f0d67","components/general/FigureFrame/FigureFrame.prompt.md":"7c0d9b6f35f1","components/general/Slide/Slide.jsx":"f77360bf86ef","components/general/Slide/Slide.d.ts":"8700f7f991ad","components/general/Slide/Slide.prompt.md":"d33aaf452f9d","components/general/StatCard/StatCard.jsx":"785d2a6311a3","components/general/StatCard/StatCard.d.ts":"29989e959ea7","components/general/StatCard/StatCard.prompt.md":"a4d573fd67c7","components/general/TitleSlide/TitleSlide.jsx":"b01dcef29cc3","components/general/TitleSlide/TitleSlide.d.ts":"7c2dfa6831d1","components/general/TitleSlide/TitleSlide.prompt.md":"03bf53445962","components/general/TocSlide/TocSlide.jsx":"78fd22d0cdc4","components/general/TocSlide/TocSlide.d.ts":"c71ccd640b07","components/general/TocSlide/TocSlide.prompt.md":"8923a5830f99","components/general/UosLogo/UosLogo.jsx":"68ee8f92ed85","components/general/UosLogo/UosLogo.d.ts":"be50a8f7b181","components/general/UosLogo/UosLogo.prompt.md":"2c77fcbf71da"},"inlinedExternals":[],"builtBy":"cc-design-sync"} */
"use strict";
var UOSSlideDS = (() => {
  var __create = Object.create;
  var __defProp = Object.defineProperty;
  var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
  var __getOwnPropNames = Object.getOwnPropertyNames;
  var __getProtoOf = Object.getPrototypeOf;
  var __hasOwnProp = Object.prototype.hasOwnProperty;
  var __esm = (fn, res, err) => function __init() {
    if (err) throw err[0];
    try {
      return fn && (res = (0, fn[__getOwnPropNames(fn)[0]])(fn = 0)), res;
    } catch (e) {
      throw err = [e], e;
    }
  };
  var __commonJS = (cb, mod) => function __require() {
    try {
      return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
    } catch (e) {
      throw mod = 0, e;
    }
  };
  var __export = (target, all) => {
    for (var name in all)
      __defProp(target, name, { get: all[name], enumerable: true });
  };
  var __copyProps = (to, from, except, desc) => {
    if (from && typeof from === "object" || typeof from === "function") {
      for (let key of __getOwnPropNames(from))
        if (!__hasOwnProp.call(to, key) && key !== except)
          __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
    }
    return to;
  };
  var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
    // If the importer is in node compatibility mode or this is not an ESM
    // file that has been converted to a CommonJS file using a Babel-
    // compatible transform (i.e. "__esModule" has not been set), then set
    // "default" to the CommonJS "module.exports" for node compatibility.
    isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
    mod
  ));
  var __toCommonJS = (mod) => __copyProps(__defProp({}, "__esModule", { value: true }), mod);

  // <define:import.meta.env>
  var init_define_import_meta_env = __esm({
    "<define:import.meta.env>"() {
    }
  });

  // shim:react-shim
  var require_react_shim = __commonJS({
    "shim:react-shim"(exports, module) {
      init_define_import_meta_env();
      var R = window.React;
      function np(p, k) {
        var o = {};
        for (var x in p) if (x !== "children") o[x] = p[x];
        if (k !== void 0) o.key = k;
        return o;
      }
      function jsx2(t, p, k) {
        var c = p && p.children;
        return c === void 0 ? R.createElement(t, np(p, k)) : R.createElement(t, np(p, k), c);
      }
      function jsxs2(t, p, k) {
        return R.createElement.apply(R, [t, np(p, k)].concat(p.children));
      }
      module.exports = R;
      module.exports.jsx = jsx2;
      module.exports.jsxs = jsxs2;
      module.exports.jsxDEV = function(t, p, k, s) {
        return (s ? jsxs2 : jsx2)(t, p, k);
      };
      module.exports.Fragment = R.Fragment;
    }
  });

  // dist/index.js
  var index_exports = {};
  __export(index_exports, {
    AccentTab: () => AccentTab,
    BulletList: () => BulletList,
    ChapterSlide: () => ChapterSlide,
    ContentSlide: () => ContentSlide,
    DataTable: () => DataTable,
    DiagonalPanel: () => DiagonalPanel,
    FigureFrame: () => FigureFrame,
    Slide: () => Slide,
    StatCard: () => StatCard,
    TitleSlide: () => TitleSlide,
    TocSlide: () => TocSlide,
    UosLogo: () => UosLogo
  });
  init_define_import_meta_env();
  var import_react = __toESM(require_react_shim(), 1);
  var import_jsx_runtime = __toESM(require_react_shim(), 1);
  var uos_logo_default = "data:image/png;base64,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";
  function classes(...names) {
    return names.filter(Boolean).join(" ");
  }
  function titleLines(value) {
    return value.split("\n").map((line, index, lines) => /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("span", { className: "uos-text-line", children: [
      line,
      index < lines.length - 1 && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("br", {})
    ] }, `${index}-${line}`));
  }
  function Slide({
    background = "white",
    children,
    className,
    style
  }) {
    const frameRef = (0, import_react.useRef)(null);
    const [scale, setScale] = (0, import_react.useState)(1);
    (0, import_react.useEffect)(() => {
      const frame = frameRef.current;
      if (!frame || typeof ResizeObserver === "undefined") {
        return void 0;
      }
      const setScaleFromWidth = (width) => {
        if (width > 0) {
          setScale(width / 1280);
        }
      };
      setScaleFromWidth(frame.getBoundingClientRect().width);
      const observer = new ResizeObserver((entries) => {
        const width = entries[0]?.contentRect.width ?? frame.getBoundingClientRect().width;
        setScaleFromWidth(width);
      });
      observer.observe(frame);
      return () => observer.disconnect();
    }, []);
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
      "div",
      {
        className: classes("uos-slide", className),
        "data-uos-scale": scale,
        ref: frameRef,
        style,
        children: /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
          "div",
          {
            className: classes(
              "uos-slide__canvas",
              `uos-slide__canvas--${background}`
            ),
            style: { transform: `scale(${scale})` },
            children
          }
        )
      }
    );
  }
  function UosLogo({
    className,
    size = 110.52,
    style,
    variant = "color"
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
      "img",
      {
        alt: "University of Seoul",
        className: classes(
          "uos-logo",
          variant === "white" && "uos-logo--white",
          className
        ),
        height: size,
        src: uos_logo_default,
        style: { ...style, width: size, height: size },
        width: size
      }
    );
  }
  function AccentTab({
    className,
    size = "lg",
    style
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
      "div",
      {
        "aria-hidden": "true",
        className: classes(
          "uos-accent-tab",
          `uos-accent-tab--${size}`,
          className
        ),
        style
      }
    );
  }
  function DiagonalPanel({
    children,
    className,
    style
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: classes("uos-diagonal-panel", className), style, children });
  }
  function TitleSlide({
    className,
    date,
    showLogo = true,
    style,
    subtitle,
    title
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(Slide, { className: classes("uos-title-slide", className), style, children: [
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)(DiagonalPanel, {}),
      /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("div", { className: "uos-title-slide__title", children: [
        /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-title-slide__title-text", children: titleLines(title) }),
        subtitle && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-title-slide__subtitle", children: subtitle })
      ] }),
      showLogo && /* @__PURE__ */ (0, import_jsx_runtime.jsx)(UosLogo, { className: "uos-title-slide__logo", variant: "white" }),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { "aria-hidden": "true", className: "uos-title-slide__date-badge" }),
      date && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-title-slide__date", children: date })
    ] });
  }
  function TocSlide({
    className,
    items,
    label = "Table of Contents",
    style
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(Slide, { className: classes("uos-toc-slide", className), style, children: [
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)(AccentTab, {}),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-toc-slide__label", children: label }),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-toc-slide__items", children: items.map((item, index) => {
        const title = typeof item === "string" ? item : item.title;
        const page = typeof item === "string" ? void 0 : item.page;
        return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("div", { className: "uos-toc-slide__item", children: [
          /* @__PURE__ */ (0, import_jsx_runtime.jsx)("span", { children: title }),
          page !== void 0 && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("span", { className: "uos-toc-slide__page", children: page })
        ] }, `${index}-${title}`);
      }) })
    ] });
  }
  function ChapterSlide({
    chapterNo,
    className,
    style,
    title
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(Slide, { className: classes("uos-chapter-slide", className), style, children: [
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)(AccentTab, {}),
      /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("div", { className: "uos-chapter-slide__title", children: [
        chapterNo !== void 0 && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("span", { className: "uos-chapter-slide__number", children: chapterNo }),
        /* @__PURE__ */ (0, import_jsx_runtime.jsx)("span", { className: "uos-chapter-slide__title-text", children: titleLines(title) })
      ] })
    ] });
  }
  function ContentSlide({
    children,
    className,
    style,
    title,
    variant = "plain"
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(Slide, { className: classes("uos-content-slide", className), style, children: [
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)(AccentTab, { size: "sm" }),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-content-slide__title", children: title }),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
        "div",
        {
          className: classes(
            "uos-content-slide__content",
            variant === "placeholder" && "uos-content-slide__content--placeholder"
          ),
          children
        }
      )
    ] });
  }
  function BulletList({
    className,
    items,
    marker = "dot",
    style
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)(
      "ul",
      {
        className: classes(
          "uos-bullet-list",
          `uos-bullet-list--${marker}`,
          className
        ),
        style,
        children: items.map((item, index) => {
          const text = typeof item === "string" ? item : item.text;
          const children = typeof item === "string" ? void 0 : item.children;
          return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("li", { children: [
            /* @__PURE__ */ (0, import_jsx_runtime.jsx)("span", { children: text }),
            children && children.length > 0 && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("ul", { className: "uos-bullet-list__children", children: children.map((child, childIndex) => /* @__PURE__ */ (0, import_jsx_runtime.jsx)("li", { children: child }, `${childIndex}-${child}`)) })
          ] }, `${index}-${text}`);
        })
      }
    );
  }
  function StatCard({
    caption,
    className,
    label,
    style,
    tone = "blue",
    value
  }) {
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(
      "div",
      {
        className: classes(
          "uos-stat-card",
          `uos-stat-card--${tone}`,
          className
        ),
        style,
        children: [
          /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-stat-card__value", children: value }),
          /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-stat-card__label", children: label }),
          caption && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-stat-card__caption", children: caption })
        ]
      }
    );
  }
  function DataTable({
    align = [],
    className,
    columns,
    rows,
    style
  }) {
    const alignment = (index) => ({
      textAlign: align[index] ?? "left"
    });
    return /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: classes("uos-data-table", className), style, children: /* @__PURE__ */ (0, import_jsx_runtime.jsxs)("table", { children: [
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("thead", { children: /* @__PURE__ */ (0, import_jsx_runtime.jsx)("tr", { children: columns.map((column, index) => /* @__PURE__ */ (0, import_jsx_runtime.jsx)("th", { style: alignment(index), children: column }, `${index}-${column}`)) }) }),
      /* @__PURE__ */ (0, import_jsx_runtime.jsx)("tbody", { children: rows.map((row, rowIndex) => /* @__PURE__ */ (0, import_jsx_runtime.jsx)("tr", { children: row.map((cell, columnIndex) => /* @__PURE__ */ (0, import_jsx_runtime.jsx)("td", { style: alignment(columnIndex), children: cell }, columnIndex)) }, rowIndex)) })
    ] }) });
  }
  function FigureFrame({
    caption,
    children,
    className,
    ratio = "16/9",
    style
  }) {
    const empty = children === void 0 || children === null || typeof children === "boolean" || children === "";
    return /* @__PURE__ */ (0, import_jsx_runtime.jsxs)(
      "figure",
      {
        className: classes(
          "uos-figure-frame",
          empty && "uos-figure-frame--empty",
          className
        ),
        style: { aspectRatio: ratio, ...style },
        children: [
          /* @__PURE__ */ (0, import_jsx_runtime.jsx)("div", { className: "uos-figure-frame__content", children }),
          caption && /* @__PURE__ */ (0, import_jsx_runtime.jsx)("figcaption", { className: "uos-figure-frame__caption", children: caption })
        ]
      }
    );
  }
  return __toCommonJS(index_exports);
})();
window.UOSSlideDS=UOSSlideDS.__dsMainNs?Object.assign({},UOSSlideDS,UOSSlideDS.__dsMainNs,{__dsMainNs:undefined}):UOSSlideDS;
