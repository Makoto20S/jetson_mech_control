const fs=require('fs'),vm=require('vm'),assert=require('assert');
const page=fs.readFileSync('tools/bench_panel/ph_exo_bench_panel.html','utf8');
const script=page.match(/<script>([\s\S]*?)<\/script>/)[1];new vm.Script(script);
const fields={};for(const m of page.matchAll(/<input[^>]*id="([^"]+)"[^>]*>/g)){const v=m[0].match(/value="([^"]*)"/);let value=v?v[1]:'';fields[m[1]]={get value(){return value},set value(x){value=String(x)}};}
const ctx={fields,Number,Math,Error,Object,Array,JSON};vm.createContext(ctx);
vm.runInContext('const ROS=false,keys=["k_psi","k_beta","b_psi","b_beta","torque_nm","current_a","travel_deg"],$=id=>fields[id];'+script.slice(script.indexOf('function params()'),script.indexOf('function controls()')),ctx);
let c=vm.runInContext('validate(params())',ctx);assert.equal(c.k_psi,20);assert.equal(c.b_beta,.3);assert(!c.amp_deg);
fields.amp_psi.value='3';assert.throws(()=>vm.runInContext('params()',ctx));fields.amp_beta.value='-2';
c=vm.runInContext('validate(params())',ctx);assert.equal(c.amp_deg[1],-2);assert.equal(c.period_s[0],6);
fields.wave.value='正弦';assert.throws(()=>vm.runInContext('validate(params())',ctx));
fields.amp_beta.value='0';assert.equal(vm.runInContext('validate(params()).wave',ctx),'sine');
fields.wave.value='分段五次';fields.amp_beta.value='-2';
fields.pose_psi.value='1';fields.pose_beta.value='20';fields.period_beta.value='4';
c=vm.runInContext('validate(params())',ctx);const saved=JSON.stringify(c);vm.runInContext('setParams('+saved+')',ctx);assert.equal(JSON.stringify(vm.runInContext('params()',ctx)),saved);
fields.pose_beta.value='-7';assert.throws(()=>vm.runInContext('validate(params())',ctx));fields.pose_beta.value='20';
fields.period_beta.value='0.9';assert.throws(()=>vm.runInContext('validate(params())',ctx));fields.period_beta.value='4';
vm.runInContext('setParams({k_psi:10,k_beta:10,b_psi:0,b_beta:0,torque_nm:10,current_a:6,travel_deg:90})',ctx);
assert.equal(fields.amp_psi.value,'');assert.equal(fields.pose_psi.value,'');assert(!vm.runInContext('validate(params())',ctx).amp_deg);
console.log('UI syntax, signed amplitude, partial input rejection, range checks, export/import and legacy reset passed');

const rosCtx={fields,Number,Math,Error,Object,Array,JSON};vm.createContext(rosCtx);
vm.runInContext('const ROS=true,keys=["k_psi","k_beta","b_psi","b_beta","torque_nm","current_a","travel_deg"],$=id=>fields[id];'+script.slice(script.indexOf('function params()'),script.indexOf('function controls()')),rosCtx);
vm.runInContext('setParams({k:[20,20],b:[.3,.3],tau:[10,10],pose_deg:[0,20],move_s:3,amp_deg:[3,-2],period_s:[6,4]})',rosCtx);
const rosSaved=JSON.stringify(vm.runInContext('validate(params())',rosCtx));
vm.runInContext('setParams('+rosSaved+')',rosCtx);assert.equal(JSON.stringify(vm.runInContext('params()',rosCtx)),rosSaved);
fields.pose_beta.value='-7';assert.throws(()=>vm.runInContext('validate(params())',rosCtx));
vm.runInContext('setParams({k:[10,10],b:[0,0],tau:[10,10]})',rosCtx);
assert.equal(fields.amp_psi.value,'');assert.equal(fields.pose_psi.value,'');assert(!vm.runInContext('validate(params())',rosCtx).amp_deg);
console.log('ROS optional reference forms, range rejection, roundtrip JSON and fixed-mode reset passed');

// Check display quality separately from reference generation and motor commands.
const display=vm.createContext({Math,Number});
vm.runInContext('const rad=180/Math.PI;'+script.slice(script.indexOf('function encoderTime'),script.indexOf('function leavePlan')),display);
const sample={observed_s:10,state_valid:true,q:[.1,.2],reference:[.3,.4],encoder_ref_deg:[90,76],
 feedback:{104:{mit_position_deg:86,mit_rx_us:9999000,mit_valid:1,mit_fault_code:0},
           105:{mit_position_deg:77,mit_rx_us:9000000,mit_valid:1,mit_fault_code:0}}};
display.r=sample;
const values=id=>Array.from(vm.runInContext('kinValues(r,'+JSON.stringify(id)+')',display));
assert.deepEqual(values('enc104'),[90,86]);assert.deepEqual(values('enc105'),[76,null]);
assert.equal(values('psi')[1],.1*(180/Math.PI));sample.state_valid=false;
assert.equal(values('beta')[1],null);
console.log('Encoder ordering, feedback freshness and invalid FK gaps passed');
