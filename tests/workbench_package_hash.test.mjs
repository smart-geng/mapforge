import test from "node:test";
import assert from "node:assert/strict";
import {createHash, randomBytes} from "node:crypto";
import {readFileSync} from "node:fs";
import vm from "node:vm";
const env={window:{},Uint32Array,Uint8Array,DataView,setTimeout};
vm.runInNewContext(readFileSync(new URL("../mapforge/workbench/static/sha256.js",import.meta.url),"utf8"),env);
const {SHA256,blob}=env.window.PackageHash;
const native=data=>createHash("sha256").update(data).digest("hex");
for(const length of [0,1,3,55,56,63,64,65,127,128,129,1000000]) {
  test(`SHA-256 padding and fragmented updates: ${length} bytes`,()=>{
    const data=Buffer.alloc(length,97),hash=new SHA256();
    for(let offset=0;offset<data.length;offset+=17)hash.update(data.subarray(offset,offset+17));
    assert.equal(hash.hex(),native(data));
    assert.throws(()=>hash.update(data),/finalized/);
  });
}
test("Blob hash reads bounded slices and yields progress without reading the whole file",async()=>{
  const data=randomBytes(9*1024*1024+67),reads=[],progress=[];
  const file={size:data.length,arrayBuffer(){throw new Error("whole-file read forbidden");},
    slice(a,b){reads.push([a,Math.min(b,data.length)]);return new Blob([data.subarray(a,b)]);}};
  assert.equal(await blob(file,(a,b)=>progress.push([a,b])),native(data));
  assert.equal(reads.length,3);
  assert.ok(reads.every(([a,b])=>b-a<=4*1024*1024));
  assert.deepEqual(progress.at(-1),[data.length,data.length]);
});
