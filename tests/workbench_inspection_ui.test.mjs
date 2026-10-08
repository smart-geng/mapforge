// Production app + inspection scripts in a minimal DOM with delayed responses.
// These contracts cover ownership/timing; they are not browser layout acceptance.
import test from "node:test";
import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createHash, webcrypto} from "node:crypto";
import vm from "node:vm";

const app = readFileSync(new URL("../mapforge/workbench/static/app.js", import.meta.url), "utf8");
const inspection = readFileSync(new URL("../mapforge/workbench/static/inspection.js", import.meta.url), "utf8");
const tick = () => new Promise(resolve => setImmediate(resolve));
const frame = label => ({display_label: label, reason: "declared source units", assumptions: []});
const measure = label => ({unit: "m", display_label: label, length: 12.5, point_count: 2, frame: frame(label)});

function project(id = "p1") {
  const objects = ["a", "b"].map((name, index) => ({id: name, role: "boundary", shape_type: 3,
    business_id: name, source_ref: {layer: "boundary", record_index: index, part_index: 0},
    raw_attributes: {}, points: [[index, 0], [index + 10, 0]]}));
  return {project_id: id, name: id, revision: 7, source_snapshot: {objects, frame: {}, issues: []},
    intents: [], candidate: {candidate_id: "candidate"}, validation: {decision: "BLOCKED"},
    status: {read_only: false, candidate_stale: false, validation_stale: false,
      can_undo: false, can_redo: false, source_issues: []}};
}

async function environment() {
  const elements = new Map(), listeners = new Map(), requests = [], downloads = [], timers = [];
  const context2d = Object.fromEntries(["setTransform", "clearRect", "beginPath", "moveTo", "lineTo", "arc",
    "closePath", "fill", "stroke", "save", "restore", "setLineDash"].map(name => [name, () => {}]));
  class Element {
    constructor(tag = "div") {Object.assign(this, {tag, children: [], textContent: "", value: "", disabled: false,
      hidden: true, open: false, attributes: {}, clientWidth: 900, clientHeight: 500,
      classList: {toggle() {}}});}
    setAttribute(key, value) {this.attributes[key] = value;}
    replaceChildren(...children) {this.children = children;}
    append(...children) {this.children.push(...children);}
    get firstChild() {return this.children[0];}
    addEventListener(type, action) {this["on" + type] = action;}
    getContext() {return context2d;}
    setPointerCapture() {}
    remove() {}
    click() {if (this.disabled) return; if (this.tag === "a") downloads.push({href: this.href, name: this.download});
      return this.onclick?.();}
  }
  const element = id => {if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id);};
  const response = body => ({ok: true, json: async () => body});
  const sandbox = {console, Blob, URLSearchParams, crypto: webcrypto, devicePixelRatio: 1,
    location: {hash: "", pathname: "/"}, history: {replaceState() {}},
    sessionStorage: {getItem: () => "test-token", setItem() {}},
    document: {getElementById: element, createElement: tag => new Element(tag),
      createTextNode: text => ({textContent: text}), body: new Element("body"),
      addEventListener: (type, handler) => listeners.set(type, handler)},
    window: {addEventListener() {}}, ResizeObserver: class {observe() {}},
    confirm: () => true,
    URL: {createObjectURL: () => "blob:test", revokeObjectURL() {}},
    setTimeout: (handler, delay) => {timers.push({handler, delay}); return timers.length;},
    fetch(url, options = {}) {
      if (url === "/api/catalog") return Promise.resolve(response({junctions: [], notice: "test"}));
      if (url === "/api/projects") return Promise.resolve(response([]));
      return new Promise((resolve, reject) => requests.push({url, options, reject,
        respond: body => resolve(response(body)), respondRaw: resolve}));
    }};
  vm.createContext(sandbox);
  const run = code => vm.runInContext(code, sandbox);
  run(app); run(inspection); await tick();
  const e = {sandbox, run, element, requests, downloads};
  e.open = (id = "p1") => {
    run(`generation++;applyProject(${JSON.stringify(project(id))},true);`);
    return requests.at(-1);
  };
  e.ready = async (id = "p1") => {e.open(id).respond(frame(id + " frame")); await tick(); run('selectObject("a");');};
  e.pendingNote = () => {element("note").value = "unconfirmed note"; element("note").oninput();};
  e.escape = () => {const event = {key: "Escape", prevented: false, preventDefault() {this.prevented = true;}};
    listeners.get("keydown")(event); return event;};
  return e;
}

test("clear and object selection do not cancel the independent frame response", async () => {
  const e = await environment(), pending = e.open();
  e.run('selectObject("a");');
  assert.equal(e.element("measure-object").disabled, true);
  e.element("clear-measure").click();
  e.run('selectObject("b");');
  pending.respond(frame("ready frame")); await tick();
  assert.equal(e.element("measure-points").disabled, false);
  assert.equal(e.element("measure-object").disabled, false);
  assert.match(e.element("measurement-result").textContent, /ready frame/);
});

for (const outcome of ["success", "error"]) {
  test(`old project frame ${outcome} cannot replace a later project`, async () => {
    const e = await environment(), old = e.open("old"), current = e.open("current");
    current.respond(frame("current frame")); await tick();
    if (outcome === "success") old.respond(frame("OLD frame")); else old.reject(new Error("OLD frame failed"));
    await tick();
    assert.match(e.element("measurement-result").textContent, /current frame/);
    assert.equal(e.element("measure-points").disabled, false);
  });
  for (const action of ["clear", "select", "project"]) {
    test(`late object ${outcome} is ignored after ${action}`, async () => {
      const e = await environment(); await e.ready();
      e.element("measure-object").click(); const old = e.requests.at(-1);
      if (action === "clear") e.element("clear-measure").click();
      if (action === "select") e.run('selectObject("b");');
      if (action === "project") {e.open("p2").respond(frame("p2 frame")); await tick();}
      const expected = e.element("measurement-result").textContent;
      if (outcome === "success") old.respond(measure("OLD object")); else old.reject(new Error("OLD object failed"));
      await tick();
      assert.equal(e.element("measurement-result").textContent, expected);
      assert.equal(e.element("measure-points").disabled, false);
    });
  }
}

test("starting a new point pair invalidates a pending previous pair", async () => {
  const e = await environment(); await e.ready(); e.element("measure-points").click();
  e.sandbox.window.Inspection.point([1, 2]); e.sandbox.window.Inspection.point([3, 4]);
  const old = e.requests.at(-1);
  e.sandbox.window.Inspection.point([5, 6]);
  old.respond(measure("OLD points")); await tick();
  assert.match(e.element("measurement-result").textContent, /已取第一点/);
  e.sandbox.window.Inspection.point([7, 8]); const latest = e.requests.at(-1);
  assert.deepEqual(JSON.parse(latest.options.body), {points: [[5, 6], [7, 8]]});
  latest.respond(measure("NEW points")); await tick();
  assert.match(e.element("measurement-result").textContent, /NEW points/);
});

for (const active of ["measurement", "picks"]) {
  test(`Escape first closes ${active} and preserves the background pending note`, async () => {
    const e = await environment(); await e.ready(); e.pendingNote();
    if (active === "measurement") e.element("measure-points").click();
    else e.sandbox.window.Inspection.pick([{id: "a"}, {id: "b"}]);
    assert.equal(e.escape().prevented, true);
    assert.equal(e.element("note").value, "unconfirmed note");
    assert.equal(e.run("pendingNote"), true);
    assert.equal(e.element("measure-points").attributes["aria-pressed"], "false");
    assert.equal(e.element("pick-panel").hidden, true);
    // Once inspection has no active interaction, the existing note Escape
    // contract remains intact rather than swallowing all future Escape keys.
    e.escape(); assert.equal(e.element("note").value, ""); assert.equal(e.run("pendingNote"), false);
  });
}

test("open modal retains Escape precedence over inspection and pending notes", async () => {
  const e = await environment(); await e.ready(); e.pendingNote(); e.element("measure-points").click();
  e.element("editing-dialog").open = true; e.escape();
  assert.equal(e.element("note").value, "unconfirmed note");
  assert.equal(e.element("measure-points").attributes["aria-pressed"], "true");
});

test("declining selection preserves the pending note and the existing measurement", async () => {
  const e = await environment(); await e.ready(); e.element("measure-object").click();
  e.requests.at(-1).respond(measure("object A")); await tick(); e.pendingNote();
  e.sandbox.confirm = () => false; e.run('selectObject("b");');
  assert.equal(e.run("selected"), "a"); assert.equal(e.element("note").value, "unconfirmed note");
  assert.match(e.element("measurement-result").textContent, /object A/);
});

const bytes = new TextEncoder().encode("actual package bytes for download contract");
const receipt = () => ({export_id: "research-test", file_name: "RESEARCH_ONLY.zip", decision: "BLOCKED",
  sha256: createHash("sha256").update(bytes).digest("hex"), size_bytes: bytes.length});
const downloadResponse = (purpose = "RESEARCH_ONLY", payload = bytes) => ({ok: true,
  headers: {get: () => purpose}, arrayBuffer: async () => payload.buffer.slice(payload.byteOffset, payload.byteOffset + payload.byteLength)});

test("research export is disabled for stale, missing, readonly and busy states", async () => {
  const e = await environment(); await e.ready(); assert.equal(e.element("research-export").disabled, false);
  for (const change of ["project.candidate=null", "project.validation=null", "project.status.read_only=true",
    "project.status.candidate_stale=true", "project.status.validation_stale=true", "busy=true"]) {
    e.run(`project=${JSON.stringify(project())};busy=false;${change};window.Inspection.buttons();`);
    assert.equal(e.element("research-export").disabled, true, change);
  }
});

for (const change of ["project.revision++", "generation++", 'project.project_id="p2"']) {
  test(`export ${change} before receipt prevents requesting the old package`, async () => {
    const e = await environment(); await e.ready(); const before = e.requests.length;
    const task = e.element("research-export").click(); e.run(change); e.requests.at(-1).respond(receipt()); await task;
    assert.equal(e.requests.length, before + 1); assert.equal(e.downloads.length, 0);
  });
  test(`export ${change} during download prevents saving old bytes`, async () => {
    const e = await environment(); await e.ready(); const task = e.element("research-export").click();
    e.requests.at(-1).respond(receipt()); await tick(); e.run(change);
    e.requests.at(-1).respondRaw(downloadResponse()); await task;
    assert.equal(e.downloads.length, 0);
  });
}

for (const invalid of ["hash", "size", "purpose"]) {
  test(`download ${invalid} mismatch never initiates a save`, async () => {
    const e = await environment(); await e.ready(); const task = e.element("research-export").click();
    const value = receipt(); if (invalid === "hash") value.sha256 = "0".repeat(64); if (invalid === "size") value.size_bytes++;
    e.requests.at(-1).respond(value); await tick();
    e.requests.at(-1).respondRaw(downloadResponse(invalid === "purpose" ? "OTHER" : "RESEARCH_ONLY")); await task;
    assert.equal(e.downloads.length, 0); assert.match(e.element("export-status").textContent, /字节校验失败/);
  });
}

test("valid checked download preserves an unconfirmed note and labels research-only intent", async () => {
  const e = await environment(); await e.ready(); e.pendingNote();
  const task = e.element("research-export").click();
  assert.equal(e.element("projects").disabled, true); assert.equal(e.element("research-export").disabled, true);
  const request = e.requests.at(-1); assert.deepEqual(JSON.parse(request.options.body), {base_revision: 7});
  request.respond(receipt()); await tick(); e.requests.at(-1).respondRaw(downloadResponse()); await task;
  assert.deepEqual(e.downloads, [{href: "blob:test", name: "RESEARCH_ONLY.zip"}]);
  assert.equal(e.element("note").value, "unconfirmed note"); assert.equal(e.run("pendingNote"), true);
  assert.match(e.element("export-status").textContent, /发起下载.*BLOCKED/); assert.equal(e.run("busy"), false);
});
