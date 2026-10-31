/* Exercise the real dashboard script with a minimal DOM, without dependencies. */
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
class Element {
  constructor(tag='div'){this.tag=tag;this.children=[];this._text='';}
  set textContent(value){this._text=String(value);this.children=[];}
  get textContent(){return this._text+this.children.map(n=>n.textContent).join('');}
  append(node){this.children.push(node);}
  replaceChildren(...nodes){this.children=nodes;this._text='';}
  addEventListener(){}
}
const elements=new Map();
const document={createElement:tag=>new Element(tag),getElementById:id=>{
  if(!elements.has(id))elements.set(id,new Element());return elements.get(id);
}};
const context={document,window:{},location:{protocol:'file:'},console,Date};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname,'dashboard.js'),'utf8'),context);
const snapshot=JSON.parse(fs.readFileSync(0,'utf8'));
assert.equal(JSON.stringify(context.window.dashboard.subscriptions),JSON.stringify([snapshot.mqtt_topic]));
for(const m of snapshot.messages) assert.equal(context.window.dashboard.receiveMqtt(m.topic,m.payload),true);
function walk(node){return [node,...node.children.flatMap(walk)];}
const rows=walk(elements.get('panels'));
for(const m of snapshot.messages){
  const row=rows.find(n=>n.title===`Display field: ${m.topic}`);
  assert.ok(row,`Missing field for ${m.topic}`);
  let value;try{value=JSON.parse(m.payload);}catch{value=m.payload;}
  assert.equal(row.children[1]._text,String(value),m.topic);
}
const heading=rows.find(n=>n.title==='Display field: Robopico/sensors/magnetometer/heading_deg');
assert.equal(heading.children[1]._text,'—','untransmitted fields must remain unavailable');
console.log(`PASS: ${snapshot.messages.length} decoded fields displayed on the correct cards`);
