// Production app + transfer scripts in a minimal DOM with a scripted server.
// These contracts cover request shape, byte checks and explicit opening; they
// are not browser layout acceptance or a real package round trip.
import test from "node:test";
import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createHash, webcrypto} from "node:crypto";
import vm from "node:vm";

const app = readFileSync(new URL("../mapforge/workbench/static/app.js", import.meta.url), "utf8");
const transfer = readFileSync(new URL("../mapforge/workbench/static/transfer.js", import.meta.url), "utf8");
const settle = async () => {for (let i = 0; i < 60; i++) await new Promise(resolve => setImmediate(resolve));};
const sha = bytes => createHash("sha256").update(bytes).digest("hex");
const PID = "a".repeat(32), IMPORTED = "b".repeat(32), UPLOAD = "c".repeat(32);
const LIMITS = {file_suffix: ".mapforge-project.zip", max_package_bytes: 1024, chunk_bytes: 4,
  source_included: false, recompute_required: true, formal_delivery: false};

function project(id = PID, revision = 7) {
  return {project_id: id, name: "工程 " + id.slice(0, 4), revision, intents: [], candidate: null, validation: null,
    source_snapshot: {objects: [], frame: {}, issues: []},
    status: {read_only: false, can_undo: false, can_redo: false, source_issues: []}};
}

async function environment() {
  const elements = new Map(), calls = [], downloads = [], routes = new Map();
  class Element {
    constructor(tag = "div") {Object.assign(this, {tag, children: [], textContent: "", value: "", disabled: false,
      hidden: true, open: false, attributes: {}, clientWidth: 900, clientHeight: 500, errors: false});
      this.classList = {toggle: (name, on) => {if (name === "error") this.errors = Boolean(on);}};}
    setAttribute(key, value) {this.attributes[key] = value;}
    replaceChildren(...children) {this.children = children;}
    append(...children) {this.children.push(...children);}
    get firstChild() {return this.children[0];}
    addEventListener(type, action) {this["on" + type] = action;}
    getContext() {return new Proxy({}, {get: () => () => {}});}
    showModal() {this.open = true;}
    close() {this.open = false;}
    remove() {}
    click() {if (this.disabled) return; if (this.tag === "a") downloads.push({href: this.href, name: this.download});
      return this.onclick?.();}
  }
  const element = id => {if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id);};
  const reply = result => result.raw || {ok: (result.status || 200) < 400, status: result.status || 200,
    json: async () => result.body, headers: {get: () => null}};
  routes.set("GET /api/catalog", () => ({body: {junctions: [], notice: "test"}}));
  routes.set("GET /api/projects", () => ({body: [project()]}));
  routes.set("GET /api/transfers", () => ({body: LIMITS}));
  const sandbox = {console, Blob, URLSearchParams, crypto: webcrypto, devicePixelRatio: 1,
    location: {hash: "", pathname: "/"}, history: {replaceState() {}},
    sessionStorage: {getItem: () => "test-token", setItem() {}},
    document: {getElementById: element, createElement: tag => new Element(tag),
      createTextNode: text => ({textContent: text}), body: new Element("body"), addEventListener() {}},
    window: {addEventListener() {}}, ResizeObserver: class {observe() {}}, confirm: () => true,
    URL: {createObjectURL: () => "blob:test", revokeObjectURL() {}},
    setTimeout: handler => {setImmediate(handler); return 1;},
    fetch(url, options = {}) {
      const method = options.method || "GET", path = url.split("?")[0];
      calls.push({method, url, path, options});
      const handler = routes.get(method + " " + path);
      if (!handler) return Promise.reject(new Error("unrouted " + method + " " + url));
      return Promise.resolve(handler({url, options})).then(reply);
    }};
  vm.createContext(sandbox);
  const run = code => vm.runInContext(code, sandbox);
  run(app); run(readFileSync(new URL("../mapforge/workbench/static/sha256.js", import.meta.url), "utf8")); run(transfer); await settle();
  const e = {sandbox, run, element, calls, downloads, routes};
  e.open = () => {run(`generation++;applyProject(${JSON.stringify(project())},true);`);};
  e.body = (method, path) => JSON.parse(calls.find(c => c.method === method && c.path === path).options.body);
  e.file = (name, bytes) => {element("transfer-file").files = [{name, size: bytes.length,
    slice: (a,b) => new Blob([bytes.slice(a,b)])}];
    element("transfer-file").onchange();};
  return e;
}

function exportRoutes(e, bytes, {header} = {}) {
  const digest = sha(bytes);
  const result = {project_id: PID, revision: 7, filename: PID + ".mapforge-project.zip", size: bytes.length, sha256: digest};
  e.routes.set("POST /api/transfers/exports", () => ({body: {job_id: "j1", state: "running", operation: "export"}}));
  e.routes.set("GET /api/transfers/jobs/j1", () => ({body: {job_id: "j1", state: "succeeded", operation: "export", result}}));
  e.routes.set("GET /api/transfers/jobs/j1/package", () => ({raw: {ok: true, status: 200, json: async () => ({}),
    blob: async () => new Blob([bytes]),
    headers: {get: key => key === "X-Content-SHA256" ? (header ?? digest) : null}}}));
}

function importRoutes(e, {job, upload} = {}) {
  let received = 0, size = 0;
  e.routes.set("POST /api/transfers/uploads", ({options}) => {size = JSON.parse(options.body).size;
    return {body: {upload_id: UPLOAD, state: "receiving", received_size: 0, expected_size: size}};});
  e.routes.set(`PUT /api/transfers/uploads/${UPLOAD}/chunks`, ({options}) => {
    received += options.body.size;
    return {body: upload?.(received) || {upload_id: UPLOAD, state: received === size ? "sealed" : "receiving",
      received_size: received, expected_size: size}};});
  e.routes.set("POST /api/transfers/imports", () => ({body: {job_id: "j2", state: "running", operation: "import"}}));
  e.routes.set("GET /api/transfers/jobs/j2", () => ({body: job || {job_id: "j2", state: "succeeded", operation: "import",
    result: {project_id: IMPORTED, revision: 9, candidate_stale: true, validation_stale: true,
      recompute_required: true, formal_delivery: false}}}));
  e.routes.set(`GET /api/projects/${IMPORTED}`, () => ({body: project(IMPORTED, 9)}));
  e.routes.set(`GET /api/projects/${IMPORTED}/jobs`, () => ({body: []}));
}

test("export sends only the open project's identity and saves verified bytes", async () => {
  const e = await environment(); e.open();
  const bytes = new TextEncoder().encode("PK stored package bytes");
  exportRoutes(e, bytes);
  assert.equal(e.element("open-transfer").disabled, false);
  e.element("open-transfer").click();
  await e.element("transfer-export").click(); await settle();
  const body = e.body("POST", "/api/transfers/exports");
  assert.deepEqual(Object.keys(body).sort(), ["base_revision", "project_id", "request_id"]);
  assert.equal(body.project_id, PID); assert.equal(body.base_revision, 7);
  const download = e.calls.find(c => c.path === "/api/transfers/jobs/j1/package");
  assert.equal(download.options.headers.Authorization, "Bearer test-token");
  assert.deepEqual(e.downloads, [{href: "blob:test", name: PID + ".mapforge-project.zip"}]);
  assert.match(e.element("transfer-export-status").textContent, /已导出.*不含原始 SHP/);
  assert.equal(e.element("transfer-export-status").errors, false);
});

for (const fault of ["header", "body"]) {
  test(`export with mismatched ${fault} hash is not saved`, async () => {
    const e = await environment(); e.open();
    const bytes = new TextEncoder().encode("PK stored package bytes");
    exportRoutes(e, bytes, fault === "header" ? {header: "0".repeat(64)} : {});
    if (fault === "body") e.routes.set("GET /api/transfers/jobs/j1", () => ({body: {job_id: "j1", state: "succeeded",
      result: {filename: "x.mapforge-project.zip", size: bytes.length, sha256: "1".repeat(64)}}}));
    await e.element("transfer-export").click(); await settle();
    assert.deepEqual(e.downloads, []);
    assert.match(e.element("transfer-export-status").textContent, /未导出：下载字节校验失败/);
    assert.equal(e.element("transfer-export-status").errors, true);
    assert.equal(e.element("transfer-export").disabled, false);
  });
}

test("failed export job explains the service reason without downloading", async () => {
  const e = await environment(); e.open();
  exportRoutes(e, new Uint8Array([1]));
  e.routes.set("GET /api/transfers/jobs/j1", () => ({body: {job_id: "j1", state: "failed",
    error: {code: "project-revision-conflict", message: "TransferRejected: 工程修订已变化，请刷新后导出"}}}));
  await e.element("transfer-export").click(); await settle();
  assert.equal(e.calls.some(c => c.path.endsWith("/package")), false);
  assert.equal(e.element("transfer-export-status").textContent, "未导出：工程修订已变化，请刷新后导出（project-revision-conflict）");
});

test("import uploads contiguous raw chunks with the file hash and waits for an explicit open", async () => {
  const e = await environment(); e.open();
  importRoutes(e);
  const bytes = new TextEncoder().encode("0123456789");
  e.file("迁移.mapforge-project.zip", bytes);
  e.element("open-transfer").click();
  await e.element("transfer-import").click(); await settle();
  assert.deepEqual(e.body("POST", "/api/transfers/uploads"),
    {name: "迁移.mapforge-project.zip", size: 10, sha256: sha(bytes)});
  const chunks = e.calls.filter(c => c.method === "PUT");
  assert.deepEqual(chunks.map(c => c.url.split("?")[1]), ["offset=0", "offset=4", "offset=8"]);
  for (const c of chunks) {
    assert.equal(c.options.headers["Content-Type"], "application/octet-stream");
    assert.equal(c.options.headers["X-Mapforge-CSRF"], "test-token");
  }
  assert.deepEqual(Object.keys(e.body("POST", "/api/transfers/imports")).sort(), ["request_id", "upload_id"]);
  assert.equal(e.element("transfer-result").hidden, false);
  assert.match(e.element("transfer-result-text").textContent, /旧候选和检查已过期，编辑能力需重新登记/);
  assert.match(e.element("transfer-import-status").textContent, /工程尚未打开/);
  // Import never replaces the current project by itself.
  assert.equal(e.run("project.project_id"), PID);
  assert.equal(e.calls.some(c => c.path === `/api/projects/${IMPORTED}`), false);
  await e.element("transfer-open").click(); await settle();
  assert.equal(e.run("project.project_id"), IMPORTED);
  assert.equal(e.element("projects").value, IMPORTED);
  assert.equal(e.element("transfer-dialog").open, false);
  assert.match(e.element("notice").textContent, /旧候选和检查已过期/);
});

test("a project without accepted results is not described as having stale ones", async () => {
  const e = await environment(); e.open();
  importRoutes(e, {job: {job_id: "j2", state: "succeeded", result: {project_id: IMPORTED, revision: 9,
    candidate_stale: false, validation_stale: false, recompute_required: true, formal_delivery: false}}});
  e.file("x.mapforge-project.zip", new TextEncoder().encode("0123"));
  await e.element("transfer-import").click(); await settle();
  assert.match(e.element("transfer-result-text").textContent, /原工程没有已接受的候选或检查/);
  assert.doesNotMatch(e.element("transfer-result-text").textContent, /已过期/);
  await e.element("transfer-open").click(); await settle();
  assert.match(e.element("notice").textContent, /原工程没有已接受的候选或检查/);
});

test("renamed or empty packages are refused before any upload", async () => {
  for (const [name, bytes] of [["project (1).zip", new Uint8Array([1])],
    ["x.mapforge-project.zip", new Uint8Array([])], ["x.mapforge-project.zip", new Uint8Array(1025)]]) {
    const e = await environment();
    importRoutes(e); e.file(name, bytes);
    await e.element("transfer-import").click(); await settle();
    assert.equal(e.calls.some(c => c.path.startsWith("/api/transfers/uploads")), false, name);
    assert.match(e.element("transfer-import-status").textContent, /^未导入：/);
  }
});

test("failed upload seal stops before import", async () => {
  const e = await environment();
  importRoutes(e, {upload: received => received === 10 ? {upload_id: UPLOAD, state: "failed", received_size: 10,
    error: {code: "upload-hash-mismatch", message: "上传字节与声明 SHA256 不一致"}} : null});
  e.file("x.mapforge-project.zip", new TextEncoder().encode("0123456789"));
  await e.element("transfer-import").click(); await settle();
  assert.equal(e.calls.some(c => c.path === "/api/transfers/imports"), false);
  assert.match(e.element("transfer-import-status").textContent, /上传校验失败.*upload-hash-mismatch/);
  assert.equal(e.element("transfer-result").hidden, true);
});

test("same-id import failure is explained and offers nothing to open, even with the dialog closed", async () => {
  const e = await environment(); e.open();
  importRoutes(e, {job: {job_id: "j2", state: "failed", result: null, error: {code: "project-already-exists",
    message: "TransferRejected: 当前 workspace 已有同 ID 工程；不重命名或覆盖"}}});
  e.file("x.mapforge-project.zip", new TextEncoder().encode("0123"));
  await e.element("transfer-import").click(); await settle();
  assert.equal(e.element("transfer-import-status").textContent,
    "未导入：当前 workspace 已有同 ID 工程；不重命名或覆盖（project-already-exists）。工作区未新增工程。");
  assert.equal(e.element("transfer-result").hidden, true);
  assert.equal(e.element("transfer-open").disabled, true);
  assert.match(e.element("notice").textContent, /工程迁移未导入：.*同 ID 工程/);
  assert.equal(e.run("project.project_id"), PID);
});

test("one transfer at a time: export and import are disabled while either runs", async () => {
  const e = await environment(); e.open();
  let release;
  exportRoutes(e, new Uint8Array([1]));
  e.routes.set("POST /api/transfers/exports", () => new Promise(resolve => {release = () =>
    resolve({body: {job_id: "j1", state: "failed", error: {code: "x", message: "stop"}}});}));
  e.file("x.mapforge-project.zip", new Uint8Array([1]));
  const pending = e.element("transfer-export").click(); await settle();
  assert.equal(e.element("transfer-export").disabled, true);
  assert.equal(e.element("transfer-import").disabled, true);
  assert.equal(e.element("transfer-file").disabled, true);
  release(); await pending; await settle();
  assert.equal(e.element("transfer-export").disabled, false);
  assert.equal(e.element("transfer-import").disabled, false);
});
