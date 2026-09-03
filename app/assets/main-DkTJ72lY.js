import{c as V}from"./platform-_s-HGKbv.js";const kt=new Map;function W(e,t){return kt.has(e)||kt.set(e,new Set),kt.get(e).add(t),()=>kt.get(e).delete(t)}function Me(e,t){const a=kt.get(e);if(a)for(const n of a)n(t)}let Qe=!1;function In(){return Qe}const Ee=[];function cs(e){for(let t=0;t<Ee.length;t++){const a=Ee[t];if(a.collect&&a.collect(e))return!0;if(a.match(e))return Ee.splice(t,1),clearTimeout(a.timer),a.resolve(a.result!==void 0?a.result:e),!0;if(t===0&&a.fail&&a.fail(e))return Ee.splice(t,1),clearTimeout(a.timer),a.reject(new Error(e)),!0}return!1}const li="Conecta el Orby para poder editar la configuración",wa=20;function _(e,{match:t,collect:a,result:n,fail:r,timeout:i=4e3}={}){return Qe?(window.orby.sendCommand(e),Me("tx",e),t?new Promise((o,c)=>{const u={match:t,collect:a,resolve:o,reject:c,result:n,fail:r};u.timer=setTimeout(()=>{const p=Ee.indexOf(u);p>=0&&Ee.splice(p,1),c(new Error(`Sin respuesta a "${e}"`))},i),Ee.push(u)}):Promise.resolve(null)):Promise.reject(new Error(li))}let Qa=null;function ds(e){Qa=e}function at(e){return Qa?Qa(e):Promise.resolve()}function us(e){window.orby.sendCommand(e),Me("tx",e)}const ci=e=>_(`SET_PROFILE:${e}`,{match:t=>t.startsWith("PROFILE:OK:")}),ps=e=>_(`SET_TIMEOUT:${e}`,{match:t=>t.startsWith("TIMEOUT:OK:")}),fs=()=>_("SAVE_STATE",{match:e=>e.startsWith("SAVE:OK"),timeout:8e3}),hs=()=>_("RESET_DEFAULTS",{match:e=>e.startsWith("RESET:OK")}),di=async(e,t,a,n)=>(await at(e),_(`SET_PSCROLL:${e}:${t}:${a}:${n?1:0}`,{match:r=>r.startsWith(`PSCROLL:OK:${e}:${t}:`)})),Vt=async(e,t,a)=>(await at(e),_(`SET_LABEL:${e}:${t}:${a}`,{match:n=>n===`LABEL:OK:${e}:${t}`})),nt=async(e,t,a,n)=>(await at(e),_(`SET_KEYMAP:${e}:${t}:${a}:${n}`,{match:r=>r===`KEYMAP:OK:${e}:${t}`})),Ft=async(e,t,a,n,r)=>(await at(e),_(`SET_ROTARY:${e}:${t}:${a}:${n}:${r}`,{match:i=>i===`ROTARY:OK:${e}:${t}`})),ui=(e,t)=>_(`SET_NAME:${e}:${t}`,{match:a=>a===`NAME:OK:${e}`}),Wt=async(e,t)=>(t!==wa&&await at(e),_(`OLED_CLEAR:${e}:${t}`,{match:a=>a.startsWith(`OLED:CLEARED:${e}:`)})),Ln=e=>e.startsWith("ERR:"),ms=(e,t,a,n,r,i=1,o=0)=>_(`SET_MACRO_STEP:${e}:${t}:${a}:${n}:${r}:${i}:${o}`,{match:c=>c===`MACRO:OK:${e}:${t}`,fail:Ln}),gs=(e,t)=>_(`MACRO_TRUNC:${e}:${t}`,{match:a=>a===`MACRO:TRUNC:OK:${e}:${t}`,fail:Ln}),ys=e=>_(`MACRO_CLEAR:${e}`,{match:t=>t===`MACRO:CLEARED:${e}`,fail:Ln});function bs(e){const t=[],a=`MACRO:${e}:STEP:`;return _(`GET_MACRO:${e}`,{collect:n=>{if(!n.startsWith(a))return!1;const[r,i,o,c,u,p]=n.slice(a.length).split(":").map(Number);return t[r]={type:i,a:o,b:c,repeat:u,gap:p},!0},match:n=>n===`MACRO:${e}:END`||n.startsWith("ERR:"),result:t})}function An(e){return _(e,{timeout:6e3,match:t=>t.startsWith("PROFILE:ADDED:")||t.startsWith("PROFILE:DELETED:")||t.startsWith("ERR:")}).then(t=>{if(t.startsWith("ERR:"))throw new Error(t.slice(4));return parseInt(t.slice(t.lastIndexOf(":")+1),10)})}const pi=()=>An("ADD_PROFILE"),vs=e=>An(`DUP_PROFILE:${e}`),$s=e=>An(`DEL_PROFILE:${e}`);function xs(){const e={};return _("GET_STATE",{collect:t=>{if(t.startsWith("STATE:PROFILE:"))return e.profile=parseInt(t.slice(14),10),!0;if(t.startsWith("STATE:PROFILES:")){const[a,n]=t.slice(15).split(":").map(Number);return e.profileCount=a,e.maxProfiles=n,!0}if(t.startsWith("STATE:TIMEOUT:"))return e.timeout=parseInt(t.slice(14),10),!0;if(t.startsWith("STATE:MODE:"))return e.mode=t.slice(11),!0;if(t.startsWith("STATE:SUPER:"))return e.superActive=t.slice(12)==="1",!0;if(t.startsWith("STATE:PAGE:")){const[a,n,r]=t.slice(11).split(":").map(Number);return e.pageIdx=a||0,e.pageCount=n||1,e.maxPages=r||1,!0}if(t.startsWith("SCROLL:OK:")){const[a,n,r,i]=t.slice(10).split(":").map(Number);return e.scroll={detentsPerRev:a,invert:!!n,hires:!!r,hiresPan:!!i},!0}return!1},match:t=>t==="STATE:END",result:e})}function ws(){const e={count:0,per:[],all:null};return _("GET_HASH",{timeout:8e3,collect:t=>{if(t.startsWith("HASH:PROFILES:"))return e.count=parseInt(t.slice(14),10),!0;if(t.startsWith("HASH:P:")){const[a,n]=t.slice(7).split(":");return e.per[parseInt(a,10)]=n,!0}return t.startsWith("HASH:ALL:")?(e.all=t.slice(9),!0):!1},match:t=>t==="HASH:END"||t.startsWith("ERR:"),result:e}).catch(()=>null)}function Za(){const e={labels:new Array(20).fill(""),keys:[],rotary:[],scroll:[{detentsPerRev:60,invert:!1},{detentsPerRev:60,invert:!1}],oledMask:0};for(let t=0;t<24;t++)e.keys.push({modifier:0,keycode:0});for(let t=0;t<16;t++)e.rotary.push({type:0,modifier:0,keycode:0});return e}const Ms=["labels","keys","rotary","scroll","oledMask"];function fi(e){for(const t of Ms)Object.defineProperty(e,t,{get(){return(e.pages[e.pageIdx]||e.pages[0])[t]},set(a){(e.pages[e.pageIdx]||e.pages[0])[t]=a},enumerable:!1,configurable:!0});return e}function hi(e){const t={idx:e,name:"",pageCount:1,maxPages:1,pageIdx:0,pages:[]},a=Za(),n=o=>{for(;t.pages.length<=o;)t.pages.push(Za());return t.pages[o]},r=(o,c)=>{if(c.startsWith("LBL:")){const u=c.slice(4),p=u.indexOf(":");return o.labels[parseInt(u.slice(0,p),10)]=u.slice(p+1),!0}if(c.startsWith("KEY:")){const[u,p,h]=c.slice(4).split(":").map(Number);return o.keys[u]={modifier:p,keycode:h},!0}if(c.startsWith("ROT:")){const[u,p,h,v]=c.slice(4).split(":").map(Number);return o.rotary[u]={type:p,modifier:h,keycode:v},!0}if(c.startsWith("SCR:")){const[u,p,h]=c.slice(4).split(":").map(Number);return o.scroll[u]={detentsPerRev:p,invert:!!h},!0}return c.startsWith("OLEDMASK:")?(o.oledMask=parseInt(c.slice(9),10),!0):!1},i=`PROF:${e}:`;return _(`GET_PROFILE:${e}`,{timeout:12e3,collect:o=>{if(!o.startsWith(i))return!1;const c=o.slice(i.length);if(c.startsWith("NAME:"))return t.name=c.slice(5),!0;if(c.startsWith("PAGES:")){const[p,h]=c.slice(6).split(":").map(Number);return t.pageCount=p||1,t.maxPages=h||1,n(t.pageCount-1),!0}const u=c.match(/^P(\d+):(.*)$/);return u?r(n(parseInt(u[1],10)),u[2]):r(a,c)},match:o=>o===`${i}END`,result:t}).then(o=>(o.pages.length||(o.pages.push(a),o.pageCount=1,o.maxPages=1),fi(o)))}function mi(e){return _(`SET_PAGE:${e}`,{match:t=>t.startsWith("PAGE:OK:")||t.startsWith("ERR:")})}function gi(e,t=!0){return _(`ADD_PAGE:${e}:${t?1:0}`,{match:a=>a.startsWith("PAGE:ADDED:")||a.startsWith("ERR:")})}function ks(e,t){return _(`DEL_PAGE:${e}:${t}`,{match:a=>a.startsWith("PAGE:DELETED:")||a.startsWith("ERR:")})}async function Tn(e,t){const a=new Uint8Array(360);let n=!1;const r=`OLEDDATA:${e}:${t}:`;return t!==wa&&await at(e),_(`GET_OLED:${e}:${t}`,{timeout:6e3,collect:i=>{if(!i.startsWith(r))return!1;const o=i.slice(r.length);if(o==="NONE"||o==="END")return!1;const c=o.indexOf(":");if(c<0)return!1;const u=parseInt(o.slice(0,c),10),p=o.slice(c+1);for(let h=0;h+1<p.length;h+=2)a[u+h/2]=parseInt(p.substr(h,2),16);return n=!0,!0},match:i=>i===`${r}END`||i===`${r}NONE`,result:null}).then(i=>n?a:null,()=>null)}function Rn(e,t,a){const n=new Uint8Array(360);let r=!1;const i=`OLEDDATA:${e}:P${t}:${a}:`;return _(`GET_OLED_PG:${e}:${t}:${a}`,{timeout:6e3,collect:o=>{if(!o.startsWith(i))return!1;const c=o.slice(i.length);if(c==="NONE"||c==="END")return!1;const u=c.indexOf(":");if(u<0)return!1;const p=parseInt(c.slice(0,u),10),h=c.slice(u+1);for(let v=0;v+1<h.length;v+=2)n[p+v/2]=parseInt(h.substr(v,2),16);return r=!0,!0},match:o=>o===`${i}END`||o===`${i}NONE`||o.startsWith("ERR:"),result:null}).then(()=>r?n:null,()=>null)}const yr=90;async function ve(e,t,a){t!==wa&&await at(e);for(let n=0;n<a.length;n+=yr){const r=a.subarray(n,Math.min(n+yr,a.length));let i="";for(const o of r)i+=o.toString(16).padStart(2,"0");await _(`OLED_CHUNK:${e}:${t}:${n}:${i}`,{match:o=>o.startsWith(`OLED:OK:${e}:${t}:${n}:`)})}}let Ja=null;function yi(e){const t=`${e?.port||""}|${e?.raw||""}`;Qe&&t===Ja||(Ja=t,Qe=!0,Me("connected",e||{}))}const Es=2e3;let br=0;function vr(){const e=Date.now();e-br<Es||(br=e,window.orby.getStatus().then(async t=>{t!=="connected"||Qe||yi(await window.orby.getDeviceInfo())}).catch(()=>{}))}function Ps(){window.orby.onConnected(e=>yi(e)),vr(),window.orby.onDisconnected(()=>{for(Qe=!1,Ja=null;Ee.length;){const e=Ee.pop();clearTimeout(e.timer),e.reject(new Error(li))}Me("disconnected")}),window.orby.onSearching(()=>Me("searching")),window.orby.onError(e=>Me("error",e)),window.orby.onData(e=>{Qe||vr(),Me("rx",e);const t=cs(e);Me(t?"response":"telemetry",e)})}const Cs=4294967295,$r=new Uint32Array([0,498536548,997073096,651767980,1994146192,1802195444,1303535960,1342533948,3988292384,4027552580,3604390888,3412177804,2607071920,2262029012,2685067896,3183342108]),Ss=new TextEncoder;class Is{constructor(){this.value=Cs}bytes(t){let a=this.value;for(let n=0;n<t.length;n++)a=(a^t[n])>>>0,a=(a>>>4^$r[a&15])>>>0,a=(a>>>4^$r[a&15])>>>0;return this.value=a,this}u8(...t){return this.bytes(Uint8Array.from(t,a=>a&255))}u32le(t){return this.u8(t,t>>>8,t>>>16,t>>>24)}text8(t){const a=new Uint8Array(8);return a.set(Ss.encode(String(t??"")).subarray(0,8)),this.bytes(a)}done(){return(this.value^4294967295)>>>0}}function xr(e){return(e>>>0).toString(16).padStart(8,"0")}function Ls(e,t){if(!e||!e.pages?.length)return null;const a=Math.max(1,Math.min(e.pageCount||1,e.pages.length)),n=new Is;n.text8(e.name),n.u8(a);for(let i=0;i<a;i++){const o=e.pages[i];if(!o)return null;for(let u=0;u<20;u++)n.text8(o.labels?.[u]??"");for(let u=0;u<24;u++){const p=o.keys?.[u]||{modifier:0,keycode:0};n.u8(p.modifier||0,p.keycode||0)}for(let u=0;u<16;u++){const p=o.rotary?.[u]||{type:0,modifier:0,keycode:0};n.u8(p.type||0,p.modifier||0,p.keycode||0)}for(let u=0;u<2;u++){const p=o.scroll?.[u]||{detentsPerRev:60,invert:!1};n.u8(p.detentsPerRev||0,p.invert?1:0)}const c=(o.oledMask||0)&1048575;n.u32le(c);for(let u=0;u<20;u++){if(!(c&1<<u))continue;const p=t(u,i);if(!p||p.length!==360)return null;n.bytes(p)}}const r=(e.pages[0]?.oledMask||0)&1<<20?1:0;if(n.u8(r),r){const i=t(20,0);if(!i||i.length!==360)return null;n.bytes(i)}return n.done()}const en=new Set,s={connected:!1,deviceInfo:null,activeProfileIdx:0,maxProfiles:8,deviceMode:"NORMAL",superActive:!1,timeout:5,scroll:{detentsPerRev:60,invert:!1,hires:!1},profiles:[],dirty:!1,syncing:!1,pageIdx:0,pageCount:1,maxPages:1};function bi(e,t=s.pageIdx){return!e||!e.pages?null:e.pages[t]||e.pages[0]||null}function D(e){return e&&e.pages?Math.max(1,e.pageCount||e.pages.length):1}function Ut(){return Math.max(1,s.maxPages||1)}function rt(){return Ut()>1}const wr=[{icon:"profiles",accent:"#8b5cf6"},{icon:"pencil",accent:"#ec4899"},{icon:"bolt",accent:"#22d3ee"},{icon:"oled",accent:"#f59e0b"},{icon:"key",accent:"#10b981"},{icon:"wheel",accent:"#f43f5e"},{icon:"text",accent:"#6366f1"},{icon:"fill",accent:"#84cc16"}];function la(e){return wr[e%wr.length]}function it(e){return en.add(e),()=>en.delete(e)}function te(){for(const e of en)try{e(s)}catch(t){console.error("Error en un suscriptor de store.notify():",t)}}function B(){s.dirty=!0,te()}function $t(e=s.activeProfileIdx){return s.profiles[e]||null}const ze=[1,2,3,4,5,6,7,8,9,0,10,0],ht=12,tn=10;function vi(e,t,a){const n=ze[t];return!n||!e?null:e.labels[n-1+(a==="super"?10:0)]||""}function F(e,t){const a=ze[e];return a?a-1+(t==="super"?10:0):-1}function le(e,t){return e+(t==="super"?12:0)}const As=8;function pe(e,t){return e+(t==="super"?As:0)}function $i(e){return e==="super"?1:0}function Nn(e,t){return e?.scroll?.[$i(t)]||{detentsPerRev:60,invert:!1}}function Ts(e,t,a){const n=Math.max(1,Math.min(e.pageCount||1,e.pages.length));for(let r=0;r<n;r++){const i=(e.pages[r]?.oledMask||0)&1048575;for(let o=0;o<20;o++){if(!(i&1<<o))continue;const c=a(t,o,r);if(!c||c.length!==360)return`${t}:${r}:${o}`}}return"(ninguno: la huella falla por otra cosa)"}function Rs(e,t,a,n){if(!e||!a?.[t]||!n)return null;const r=Ls(e,(i,o)=>n(t,i,o));return r===null||xr(r)!==a[t]?(console.debug(`[sync] perfil ${t} hay que releerlo:`,r===null?`falta el icono ${Ts(e,t,n)}`:`${xr(r)} != ${a[t]}`),null):(e.idx=t,e)}async function fe({expected:e=null,iconOf:t=null,onProgress:a=null}={}){s.syncing=!0,te();const n=[];try{const r=await xs();let i=s.profiles.length||4;r&&(Number.isInteger(r.profile)&&(s.activeProfileIdx=r.profile),Number.isInteger(r.profileCount)&&(i=r.profileCount),Number.isInteger(r.maxProfiles)&&(s.maxProfiles=r.maxProfiles),Number.isInteger(r.timeout)&&(s.timeout=r.timeout),typeof r.superActive=="boolean"&&(s.superActive=r.superActive),r.mode&&(s.deviceMode=r.mode),r.scroll&&(s.scroll=r.scroll),Number.isInteger(r.pageIdx)&&(s.pageIdx=r.pageIdx),Number.isInteger(r.pageCount)&&(s.pageCount=r.pageCount),Number.isInteger(r.maxPages)&&(s.maxPages=r.maxPages));const o=s.profiles,c=[];for(let u=0;u<i;u++){const p=Rs(o[u],u,e,t);if(p){c.push(p);continue}a?.(u,i),c.push(await hi(u)),n.push(u)}s.profiles=c,s.activeProfileIdx>=c.length&&(s.activeProfileIdx=0);for(const u of c)u.pageIdx=u.idx===s.activeProfileIdx?Math.min(s.pageIdx,D(u)-1):0;s.pageCount=D(c[s.activeProfileIdx]),s.dirty=!1}finally{s.syncing=!1,te()}return n}async function xi(e){const t=$t(),a=D(t);if(!(e<0||e>=a)&&(t&&(t.pageIdx=e),s.pageIdx=e,te(),!!In()))try{await mi(e)}catch(n){throw await fe(),n}}async function Ze(e,t,a=!0){if(!In())return;const n=s.profiles[e];if(!n)return;const r=Math.min(Math.max(t||0,0),D(n)-1);if(e!==s.activeProfileIdx){if(r!==0)throw new Error("Activa este perfil en el teclado para editar esta página");return}s.pageIdx!==r&&(await mi(r),s.pageIdx=r,a&&(n.pageIdx=r))}let ut=null;async function On(e,t,a){if(ut)return a();const n=s.pageIdx;ut={profile:e,page:t};try{return await Ze(e,t,!1),await a()}finally{if(ut=null,s.pageIdx!==n)try{await Ze(e,n,!1)}catch{}}}function Ns(e){if(ut&&ut.profile===e)return Ze(e,ut.page,!1);const t=s.profiles[e];return Ze(e,t&&t.pageIdx||0)}ds(Ns);function Os(e,t,a){if(!Number.isInteger(e)||!Number.isInteger(t)||s.activeProfileIdx===e&&s.pageIdx===t&&s.pageCount===a)return!1;const n=s.profiles[s.activeProfileIdx],r=s.profiles[e];return s.activeProfileIdx=e,s.pageIdx=t,Number.isInteger(a)&&a>0&&(s.pageCount=a),n&&n!==r&&(n.pageIdx=0),r&&(Number.isInteger(a)&&a>0&&(r.pageCount=a),r.pageIdx=Math.min(Math.max(t,0),D(r)-1)),te(),!0}async function qs(){const e=$t();return!e||D(e)>=Ut()?!1:(await gi(e.idx,!1),await fe(),B(),!0)}async function _s(e){const t=$t();return!t||D(t)<=1?!1:(await ks(t.idx,e),await fe(),B(),!0)}function wi(){const e=$t();return{...Nn(e,s.superActive?"super":"normal"),hires:s.scroll.hires}}const w=72,S=40,Mi=S/8,zs=w*Mi;function Gt(){return new Uint8Array(zs)}function mt(e,t,a){return t<0||a<0||t>=w||a>=S?0:e[(a>>3)*w+t]>>(a&7)&1}function ne(e,t,a,n){if(t<0||a<0||t>=w||a>=S)return;const r=(a>>3)*w+t,i=1<<(a&7);n?e[r]|=i:e[r]&=~i}function Bs(e){for(let t=0;t<e.length;t++)e[t]=~e[t]&255}function ki(e){e.fill(0)}function js(e,t,a,n){const r=mt(e,t,a);if(r===n)return;const i=[[t,a]];for(;i.length;){const[o,c]=i.pop();o<0||c<0||o>=w||c>=S||mt(e,o,c)===r&&(ne(e,o,c,n),i.push([o+1,c],[o-1,c],[o,c+1],[o,c-1]))}}function Ds(e,t,a,n,r,i){const o=Math.abs(n-t),c=t<n?1:-1,u=-Math.abs(r-a),p=a<r?1:-1;let h=o+u;for(;ne(e,t,a,i),!(t===n&&a===r);){const v=2*h;v>=u&&(h+=u,t+=c),v<=o&&(h+=o,a+=p)}}function Ei(e){for(let t=0;t<w;t++)ne(e,t,0,1),ne(e,t,S-1,1);for(let t=0;t<S;t++)ne(e,0,t,1),ne(e,w-1,t,1)}function Hs(e,t){if(t<=0)return e;const a=new Float32Array(e.length),n=new Float32Array(e.length);for(let r=0;r<S;r++)for(let i=0;i<w;i++){let o=0,c=0;for(let u=-t;u<=t;u++){const p=i+u;p<0||p>=w||(o+=e[r*w+p],c++)}a[r*w+i]=o/c}for(let r=0;r<w;r++)for(let i=0;i<S;i++){let o=0,c=0;for(let u=-t;u<=t;u++){const p=i+u;p<0||p>=S||(o+=a[p*w+r],c++)}n[i*w+r]=o/c}return n}function Vs(e,{threshold:t=128,dither:a=!1,blur:n=0,invert:r="none",bounds:i=null}={}){const{data:o}=e.getImageData(0,0,w,S),c=Gt(),u=r==="colors";let p=new Float32Array(w*S);for(let h=0;h<w*S;h++){const v=h*4,y=o[v+3]/255,m=.299*o[v]+.587*o[v+1]+.114*o[v+2];p[h]=(u?255-m:m)*y}if(p=Hs(p,n),r==="box"&&i){const h=Math.max(0,Math.floor(i.x)),v=Math.max(0,Math.floor(i.y)),y=Math.min(w,Math.ceil(i.x+i.width)),m=Math.min(S,Math.ceil(i.y+i.height));for(let k=v;k<m;k++)for(let N=h;N<y;N++){const H=k*w+N;p[H]=255-p[H]}}if(a)for(let h=0;h<S;h++)for(let v=0;v<w;v++){const y=h*w+v,m=p[y],k=m<t?0:255;p[y]=k;const N=m-k;v+1<w&&(p[y+1]+=N*7/16),v>0&&h+1<S&&(p[y+w-1]+=N*3/16),h+1<S&&(p[y+w]+=N*5/16),v+1<w&&h+1<S&&(p[y+w+1]+=N*1/16)}for(let h=0;h<S;h++)for(let v=0;v<w;v++)ne(c,v,h,p[h*w+v]>=t?1:0);return c}function qn(){const e=document.createElement("canvas");return e.width=w,e.height=S,e.getContext("2d",{willReadFrequently:!0})}async function Fs(e){const t=await createImageBitmap(e);return{kind:"image",bitmap:t,naturalWidth:t.width,naturalHeight:t.height}}const Ws=256;function Pi(e,{size:t=Ws}={}){return new Promise((a,n)=>{const r=new Image;r.width=t,r.height=t,r.onload=()=>a({kind:"image",bitmap:r,naturalWidth:t,naturalHeight:t,crisp:!0}),r.onerror=()=>n(new Error("SVG no válido")),r.src=`data:image/svg+xml;charset=utf-8,${encodeURIComponent(e)}`})}function _n(e,{fontSize:t=16,bold:a=!0,font:n="Segoe UI"}={}){return{kind:"text",text:String(e),fontSize:t,bold:a,font:n}}function zn(e){return`${e.bold?"700 ":""}${e.fontSize}px "${e.font}", sans-serif`}function Bn(e){return e.text.split(`
`).slice(0,4)}function jn(e){if(e.kind==="image")return{width:e.naturalWidth,height:e.naturalHeight};const t=qn();t.font=zn(e);const a=Bn(e);return{width:Math.max(1,...a.map(r=>t.measureText(r).width)),height:a.length*(e.fontSize+2)}}function ot(e){return e._ink||(e._ink=e.kind==="text"?Us(e):Ks(e)),e._ink}function Us(e){const t=qn();t.font=zn(e),t.textAlign="left",t.textBaseline="top";const a=e.fontSize+2;let n=1/0,r=1/0,i=-1/0,o=-1/0;if(Bn(e).forEach((c,u)=>{if(!c.trim())return;const p=t.measureText(c),h=u*a;n=Math.min(n,-(p.actualBoundingBoxLeft||0)),i=Math.max(i,p.actualBoundingBoxRight??p.width),r=Math.min(r,h-(p.actualBoundingBoxAscent||0)),o=Math.max(o,h+(p.actualBoundingBoxDescent||0))}),!(i>n)||!(o>r)){const{width:c,height:u}=jn(e);return{x:0,y:0,width:c,height:u}}return{x:n,y:r,width:i-n,height:o-r}}const Gs=512;function Ks(e){const t=e.naturalWidth,a=e.naturalHeight,n={x:0,y:0,width:t,height:a},r=Math.min(1,Gs/Math.max(t,a)),i=Math.max(1,Math.round(t*r)),o=Math.max(1,Math.round(a*r)),c=document.createElement("canvas");c.width=i,c.height=o;const u=c.getContext("2d",{willReadFrequently:!0});u.drawImage(e.bitmap,0,0,i,o);let p;try{p=u.getImageData(0,0,i,o).data}catch{return n}let h=i,v=o,y=-1,m=-1;for(let H=0;H<o;H++)for(let we=0;we<i;we++)p[(H*i+we)*4+3]<8||(we<h&&(h=we),we>y&&(y=we),H<v&&(v=H),H>m&&(m=H));if(y<0)return n;const k=t/i,N=a/o;return{x:h*k,y:v*N,width:(y-h+1)*k,height:(m-v+1)*N}}function Dn(e){const t=ot(e);return!t.width||!t.height?1:Math.min((w-2)/t.width,(S-2)/t.height)}function Xs(e,t,a){const{width:n,height:r}=jn(t),i=n*a.scale,o=r*a.scale;if(t.kind==="image")return e.imageSmoothingEnabled=!0,e.imageSmoothingQuality="high",e.drawImage(t.bitmap,a.x,a.y,i,o),{x:a.x,y:a.y,width:i,height:o};e.save(),e.fillStyle="#fff",e.textAlign="left",e.textBaseline="top",e.translate(a.x,a.y),e.scale(a.scale,a.scale),e.font=zn(t);const c=t.fontSize+2;return Bn(t).forEach((u,p)=>e.fillText(u,0,p*c)),e.restore(),{x:a.x,y:a.y,width:i,height:o}}function Ma(e,t){const a=ot(e);return{x:t.x+a.x*t.scale,y:t.y+a.y*t.scale,width:a.width*t.scale,height:a.height*t.scale}}function Kt(e,t){const a=qn();return Xs(a,e,t),Vs(a,{threshold:t.threshold,dither:t.dither,blur:t.blur,invert:t.invert,bounds:Ma(e,t)})}function Ys(e){let t=w,a=S,n=-1,r=-1;for(let i=0;i<S;i++)for(let o=0;o<w;o++)mt(e,o,i)&&(o<t&&(t=o),o>n&&(n=o),i<a&&(a=i),i>r&&(r=i));return n<0?null:{x:t,y:a,width:n-t+1,height:r-a+1}}function Hn(e,t,a="replace"){const n=Gt();for(let r=0;r<n.length;r++)n[r]=a==="merge"?e[r]|t[r]:t[r];return n}function an(e,t,{zoom:a=8,grid:n=!0,color:r="#e9e2ff",outline:i=null}={}){t.width=w*a,t.height=S*a;const o=t.getContext("2d");o.fillStyle="#05050a",o.fillRect(0,0,t.width,t.height),o.fillStyle=r;for(let c=0;c<S;c++)for(let u=0;u<w;u++)mt(e,u,c)&&o.fillRect(u*a,c*a,a,a);if(n&&a>=5){o.strokeStyle="rgba(255,255,255,0.07)",o.lineWidth=1,o.beginPath();for(let c=0;c<=w;c++)o.moveTo(c*a+.5,0),o.lineTo(c*a+.5,t.height);for(let c=0;c<=S;c++)o.moveTo(0,c*a+.5),o.lineTo(t.width,c*a+.5);o.stroke(),o.strokeStyle="rgba(139,92,246,0.25)",o.beginPath();for(let c=1;c<Mi;c++)o.moveTo(0,c*8*a+.5),o.lineTo(t.width,c*8*a+.5);o.stroke()}if(i){o.save(),o.strokeStyle="#f59e0b",o.lineWidth=2,o.setLineDash([6,4]),o.strokeRect(i.x*a,i.y*a,i.width*a,i.height*a),o.setLineDash([]),o.fillStyle="#f59e0b";const c=Math.max(5,Math.min(10,a)),u=[[i.x,i.y],[i.x+i.width,i.y],[i.x,i.y+i.height],[i.x+i.width,i.y+i.height]];for(const[p,h]of u)o.fillRect(p*a-c/2,h*a-c/2,c,c);o.restore()}}const j=new Map,nn=new Set;let It=null,He=null,aa=!1;function Qs(e){const t=s.profiles[e];return t&&t.pageIdx||0}function ue(e,t,a=Qs(e)){return`${e}:${a}:${t}`}function ye(e,t,a){return j.get(ue(e,t,a))??null}function Vn(e){return ye(e,20,0)}function Be(e,t,a,n){j.set(ue(e,t,n),a),se()}function Zs(e){let t="";for(const a of e)t+=a.toString(16).padStart(2,"0");return t}function Js(e){const t=new Uint8Array(e.length/2);for(let a=0;a<t.length;a++)t[a]=parseInt(e.substr(a*2,2),16);return t}function el(){const e={};for(const[t,a]of j)a&&(e[t]=Zs(a));return e}function tl(e){for(const[t,a]of Object.entries(e||{}))typeof a=="string"&&a.length&&j.set(t,Js(a));se()}function ka(){j.clear(),It=null,He=null,se()}function Ci(e){const t=`${e}:`;for(const a of[...j.keys()])a.startsWith(t)&&j.delete(a);se()}function al(e){for(const t of[...j.keys()])parseInt(t,10)>=e&&j.delete(t);se()}function nl(){return It!==null||aa}function Fn(e){return nn.add(e),()=>nn.delete(e)}function se(){for(const e of nn)e()}async function De(e){if(s.connected){if(It!==null){He=e;return}It=e,se();try{const t=s.profiles[e];if(!t)return;const a=t.pageIdx||0,n=()=>s.profiles[e]!==t||(t.pageIdx||0)!==a;for(let i=0;i<20;i++){if(n()){He=e;return}if(j.has(ue(e,i,a)))continue;if(!(t.oledMask&1<<i)){j.set(ue(e,i,a),null);continue}const o=await Tn(e,i);if(n()){He=e;return}j.set(ue(e,i,a),o),se()}const r=ue(e,20,0);if(!j.has(r))if(!((t.pages[0]?.oledMask||0)&1<<20))j.set(r,null);else{const i=await Rn(e,0,20);n()||(j.set(r,i),se())}}finally{if(It=null,se(),He!==null){const t=He;He=null,De(t)}}}}const rl=10;async function il(e=()=>{}){if(aa)return;const t=[];for(const n of s.profiles){const r=D(n);for(let o=0;o<r;o++){const c=n.pages?.[o]?.oledMask||0;for(let u=0;u<20;u++){const p=ue(n.idx,u,o);if(!j.has(p)){if(!(c&1<<u)){j.set(p,null);continue}t.push({profile:n.idx,page:o,slot:u,key:p})}}}const i=ue(n.idx,20,0);j.has(i)||((n.pages[0]?.oledMask||0)&1<<20?t.push({profile:n.idx,page:0,slot:20,key:i}):j.set(i,null))}if(se(),!t.length)return;aa=!0;let a=0;try{for(const n of t){if(!s.connected)return;j.set(n.key,await Rn(n.profile,n.page,n.slot)),a++,a%rl===0&&se(),e(a,t.length)}}finally{aa=!1,se()}}function Xt(e=document){e.querySelectorAll("[data-bmp]").forEach(t=>{const a=j.get(t.dataset.bmp);a&&an(a,t,{zoom:1,grid:!1})})}const Mr={dashboard:'<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',profiles:'<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M9 9v12"/>',wheel:'<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="2.4" fill="currentColor" stroke="none"/>',oled:'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M6 15l3.5-4 2.5 3 2-2.5L18 15"/>',settings:'<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2"/><circle cx="15" cy="12" r="2"/><circle cx="8" cy="18" r="2"/>',console:'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>',minus:'<path d="M5 12h14"/>',plus:'<path d="M12 5v14M5 12h14"/>',fit:'<path d="M3 8V4h4M21 8V4h-4M3 16v4h4M21 16v4h-4"/><rect x="8" y="9" width="8" height="6" rx="1"/>',up:'<path d="M6 15l6-6 6 6"/>',down:'<path d="M6 9l6 6 6-6"/>',square:'<rect x="5" y="5" width="14" height="14" rx="1"/>',select:'<rect x="4" y="4" width="16" height="16" rx="1" stroke-dasharray="3 3"/>',close:'<path d="M6 6l12 12M18 6L6 18"/>',save:'<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>',refresh:'<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>',trash:'<path d="M3 6h18M8 6V4h8v2M6 6l1 14h10l1-14"/>',upload:'<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 3v13M7 8l5-5 5 5"/>',pencil:'<path d="M17 3l4 4L8 20l-5 1 1-5z"/>',eraser:'<path d="M4 16L13 7l6 6-6 6H7z"/><path d="M9 21h11"/>',fill:'<path d="M4 12L12 4l8 8-8 8z"/><path d="M19 15c1.5 2 2 2.8 2 3.6a2 2 0 0 1-4 0c0-.8.5-1.6 2-3.6z"/>',invert:'<circle cx="12" cy="12" r="9"/><path d="M12 3a9 9 0 0 1 0 18z" fill="currentColor" stroke="none"/>',text:'<path d="M4 6V4h16v2M12 4v16M8 20h8"/>',check:'<path d="M4 12l5 5L20 6"/>',key:'<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>',lock:'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/>',bolt:'<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>',sun:'<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',moon:'<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>',info:'<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',plug:'<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>',reset:'<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>',download:'<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 16V3M7 11l5 5 5-5"/>',copy:'<rect x="8" y="8" width="12" height="13" rx="1.5"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>',paste:'<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>'};function f(e,t=20){const a=Mr[e]||Mr.info;return`<svg class="icn" width="${t}" height="${t}" viewBox="0 0 24 24" fill="none"
    stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"
    aria-hidden="true">${a}</svg>`}function ol(e=document){e.querySelectorAll("[data-icon]").forEach(t=>{const a=parseInt(t.getAttribute("data-icon-size")||"20",10);t.innerHTML=f(t.getAttribute("data-icon"),a),t.removeAttribute("data-icon")})}const sl={success:"check",error:"close",info:"info"};function g(e,t="success",a=2600){let n=document.getElementById("toast-host");n||(n=document.createElement("div"),n.id="toast-host",document.body.appendChild(n));const r=document.createElement("div");r.className=`toast toast-${t}`,r.innerHTML=`${f(sl[t]||"info",16)}<span>${e}</span>`,n.appendChild(r),setTimeout(()=>{r.classList.add("leaving"),setTimeout(()=>r.remove(),200)},a)}function Y(){return s.connected?!0:(g("Conecta el Orby por USB para poder editar la configuración","error",4e3),!1)}let Si=null;function ll(e){Si=e}function rn(e,t){Si?.(e,t)}const Lt=4096,$e={invert:!0,offsetDeg:62,marker:"dot"};let Nt=null,Wn=0,na=0;const on=new Set;function Ii(e){return on.add(e),()=>on.delete(e)}async function cl(){try{const e=await window.orby.getConfig();Object.assign($e,e.wheelDial||{})}catch{}W("telemetry",e=>{e.startsWith("WHEEL:")&&dl(e)})}function Un(e){Object.assign($e,e),window.orby.setConfig({wheelDial:{...$e}}),Li()}function dl(e){const[t,a]=e.slice(6).split(":"),n=parseInt(t,10);if(a!==void 0){const r=parseInt(a,10);if(!Number.isFinite(r))return;Nt=ul(r)/Lt*360}else if(Number.isFinite(n))Wn+=n/Lt*360;else return;Li()}function ul(e){return(e%Lt+Lt)%Lt}function pl(){const e=Nt!==null?Nt:Wn;return($e.invert?e:-e)+$e.offsetDeg}function Li(){let e=(pl()-na)%360;if(Number.isFinite(e)){e>180&&(e-=360),e<-180&&(e+=360),na+=e;for(const t of on)t(na)}}function Gn(){return na}function fl(){const e=Nt!==null?Nt:Wn,t=$e.invert?e:-e;Un({offsetDeg:Kn(-t)})}function Kn(e){return(e%360+360)%360}function Xn(e){return`<div class="dial-marker ${$e.marker==="line"?"as-line":"as-dot"}" id="${e}"></div>`}const X=[];let z=null,Ai=null,Ti=!1;const sn=new Set;function Ri(e){return sn.add(e),()=>sn.delete(e)}function ce(e="data"){for(const t of sn)t(e)}function Ae(){return window.orby.setConfig({profileVariants:X})}function hl(e){return Array.isArray(e.matches)||(e.matches=e.match?[e.match]:[]),delete e.match,e.keys=e.keys||{},e.rotary=e.rotary||{},e.labels=e.labels||{},Number.isInteger(e.page)||(e.page=0),e}async function ml(){try{const e=await window.orby.getConfig();X.push(...(e.profileVariants||[]).map(hl))}catch{}W("disconnected",()=>{z=null,ce("applied")})}function gl(e,t){return X.filter(a=>a.profile===e&&(t===void 0||a.page===t))}function st(e){return X.find(t=>t.id===e)||null}function yl(){return z?.v||null}function Te(e){return z?.v.id===e}function he(e,t,a){return e?.[t]?.[a]??null}function Ot(e){return e?Object.keys(e.keys||{}).length+Object.keys(e.rotary||{}).length+Object.keys(e.labels||{}).length:0}function bl(e,{page:t=0,name:a,matches:n=[],field:r="any"}={}){const i={id:`v${Date.now()}${Math.random().toString(36).slice(2,6)}`,profile:e,page:t,name:a||"Variación",matches:n.filter(Boolean),field:r,keys:{},rotary:{},labels:{}};return X.push(i),Ae(),ce(),i}function Ni(e,t){const a=st(e),n=(t||"").trim().toLowerCase();return!a||!n||a.matches.includes(n)?!1:(a.matches.push(n),Ae(),ce(),!0)}function vl(e,t){const a=st(e);if(!a)return;const n=a.matches.indexOf(t);n>=0&&a.matches.splice(n,1),Ae(),ce()}function kr(e,t){const a=st(e);a&&(Object.assign(a,t),Ae(),ce())}async function $l(e){Te(e)&&await xt();const t=X.findIndex(a=>a.id===e);t>=0&&X.splice(t,1),Ae(),ce()}function Yn(e,t,a,n){const r=st(e);r&&(r[t]=r[t]||{},r[t][a]=n,Ae(),ce())}function Er(e,t,a){const n=st(e);n?.[t]&&(delete n[t][a],Ae(),ce())}function xl(e,t){for(let a=X.length-1;a>=0;a--){const n=X[a];n.profile===e&&(n.page===t?X.splice(a,1):n.page>t&&n.page--)}Ae(),ce()}function wl(e){for(let t=X.length-1;t>=0;t--)X[t].profile===e?X.splice(t,1):X[t].profile>e&&X[t].profile--;Ae(),ce()}function Ml(e){Ai=e}function ln(e){Ti=e}function kl(e,t){return e?t==="title"?(e.title||"").toLowerCase():t==="process"?(e.process||"").toLowerCase():`${e.process||""} ${e.title||""}`.toLowerCase():""}function El(e,t,a){for(const n of X){if(n.profile!==e||n.page!==t)continue;const r=kl(a,n.field);for(const i of n.matches||[])if(i&&r.includes(i))return n}return null}async function Oi(e,t,a){const n=s.profiles[e.profile];if(!n)return;const r=bi(n,t);r&&await On(e.profile,t,async()=>{for(const[i,o]of Object.entries(e.keys||{})){const c=a?r.keys[i]||{modifier:0,keycode:0}:o;await nt(e.profile,Number(i),c.modifier,c.keycode)}for(const[i,o]of Object.entries(e.rotary||{})){const c=a?r.rotary[i]||{type:0,modifier:0,keycode:0}:o;await Ft(e.profile,Number(i),c.type,c.modifier,c.keycode)}for(const[i,o]of Object.entries(e.labels||{})){const c=a?r.labels[i]||"":o;await Vt(e.profile,Number(i),c)}})}function qt(){return s.pageIdx||0}async function qi(e){const t=st(e);if(!t||!s.connected||t.profile!==s.activeProfileIdx||t.page!==qt())return;z&&z.v.id!==t.id&&await xt();const a=t.page;try{await Oi(t,a,!1),z={v:t,page:a},ce("applied")}catch{}}async function Ea(e,t){if(!z||z.page!==qt())return;const{v:a,page:n}=z,r=a[e]?.[t];if(r)try{await On(a.profile,n,async()=>{e==="keys"&&await nt(a.profile,Number(t),r.modifier,r.keycode),e==="rotary"&&await Ft(a.profile,Number(t),r.type,r.modifier,r.keycode),e==="labels"&&await Vt(a.profile,Number(t),r)})}catch{}}async function Pa(e,t,a,n){if(!Te(e)||!s.connected)return;const{v:r,page:i}=z;await On(r.profile,i,async()=>{t==="keys"&&await nt(r.profile,Number(a),n.modifier,n.keycode),t==="rotary"&&await Ft(r.profile,Number(a),n.type,n.modifier,n.keycode),t==="labels"&&await Vt(r.profile,Number(a),n)})}async function za(e,t,a){if(!Te(e)||!s.connected)return;const n=s.profiles[z.v.profile],r=n&&bi(n,z.page);if(!r)return;const i=t==="keys"?{modifier:0,keycode:0}:t==="rotary"?{type:0,modifier:0,keycode:0}:"",o=t==="labels"?r.labels:r[t];await Pa(e,t,a,o?.[a]??i)}async function xt(){if(!z)return;const{v:e,page:t}=z;z=null;const a=e.profile===s.activeProfileIdx||t===0;if(s.connected&&a)try{await Oi(e,t,!0)}catch{}ce("applied")}async function _i(){if(!s.connected)return;const e=Ti?El(s.activeProfileIdx,qt(),Ai):null;e?.id===z?.v.id&&z?.page===qt()||(z&&await xt(),e&&await qi(e.id))}async function Pl(){s.connected&&(z&&(z.page!==qt()||z.v.profile!==s.activeProfileIdx)&&await xt(),await _i())}async function Cl(e){const t=z;t&&await xt();try{return await e()}finally{t&&await qi(t.v.id)}}function zi(e,t,a,n){const r=le(a,n);return he(t,"keys",r)??e.keys[r]??{modifier:0,keycode:0}}function Pe(e,t,a,n){const r=pe(a,n);return he(t,"rotary",r)??e.rotary?.[r]??{type:0,modifier:0,keycode:0}}function Bi(e,t,a,n){const r=F(a,n);return r<0?"":he(t,"labels",r)??e.labels[r]??""}const Qn="orby-backup",ji=4;function ca(e){let t="";for(const a of e)t+=a.toString(16).padStart(2,"0");return t}function Pr(e){const t=new Uint8Array(e.length/2);for(let a=0;a<t.length;a++)t[a]=parseInt(e.substr(a*2,2),16);return t}async function Sl(e=()=>{}){if(!s.connected)throw new Error("Teclado no conectado");const t=s.profiles.length||1,a={format:Qn,version:ji,savedAt:new Date().toISOString(),firmware:s.deviceInfo?.fw||null,timeout:s.timeout,activeProfile:s.activeProfileIdx,profiles:[]};for(let n=0;n<t;n++){e(`Leyendo perfil ${n+1} de ${t}…`);const r=await hi(n),i={};for(let c=0;c<20;c++){if(!(r.oledMask&1<<c))continue;e(`Perfil ${n+1}: icono ${c+1} de 20…`);const u=await Tn(n,c);u&&(i[c]=ca(u))}let o=null;if((r.pages[0]?.oledMask||0)&1<<20){e(`Perfil ${n+1}: icono del perfil…`);const c=await Rn(n,0,20);c&&(o=ca(c))}a.profiles.push({name:r.name,labels:r.labels,keys:r.keys,rotary:r.rotary,scroll:r.scroll,pageCount:r.pageCount||1,pages:(r.pages||[]).map(c=>({labels:c.labels,keys:c.keys,rotary:c.rotary,scroll:c.scroll,oledMask:c.oledMask})),iconsPage:r.pageIdx||0,icons:i,profileIcon:o})}return a}function Il(){return{format:Qn,version:ji,savedAt:new Date().toISOString(),firmware:s.deviceInfo?.fw||null,timeout:s.timeout,activeProfile:s.activeProfileIdx,maxProfiles:s.maxProfiles,profiles:s.profiles.map(e=>({name:e.name,labels:e.labels,keys:e.keys,rotary:e.rotary,scroll:e.scroll,pageCount:e.pageCount||1,maxPages:e.maxPages||1,pages:(e.pages||[]).map(t=>({labels:t.labels,keys:t.keys,rotary:t.rotary,scroll:t.scroll,oledMask:t.oledMask})),iconsPage:e.pageIdx||0,icons:Ll(e.idx,e.pageIdx||0),profileIcon:Al(Vn(e.idx))}))}}function Ll(e,t){const a={};for(let n=0;n<20;n++){const r=ye(e,n,t);r&&(a[n]=ca(r))}return a}function Al(e){return e?ca(e):null}async function Tl(e,t=()=>{}){if(e?.format!==Qn)throw new Error("El archivo no es una copia de Orby");if(!s.connected)throw new Error("Teclado no conectado");const a=Math.min(s.maxProfiles,e.profiles.length);for(;s.profiles.length<a;)t(`Creando perfil ${s.profiles.length+1}…`),await pi(),await fe();for(let n=0;n<a;n++){const r=e.profiles[n];t(`Escribiendo perfil ${n+1} de ${a}…`),r.name&&await ui(n,r.name);const i=Math.min(r.pageCount||1,s.maxPages||1);for(;D(s.profiles[n])<i;)t(`Perfil ${n+1}: creando página ${D(s.profiles[n])+1}…`),await gi(n,!1),await fe();for(let u=0;u<20;u++)await Vt(n,u,r.labels?.[u]??"");for(let u=0;u<24;u++){const p=r.keys?.[u]||{modifier:0,keycode:0};await nt(n,u,p.modifier,p.keycode)}for(let u=0;u<16;u++){const p=r.rotary?.[u];p&&await Ft(n,u,p.type,p.modifier,p.keycode)}for(let u=0;u<2;u++){const p=r.scroll?.[u]??e.scroll;p?.detentsPerRev&&await di(n,u,p.detentsPerRev,p.invert)}const o=r.icons||{},c=Object.keys(o);for(let u=0;u<c.length;u++){const p=Number(c[u]);t(`Perfil ${n+1}: icono ${u+1} de ${c.length}…`),await ve(n,p,Pr(o[p]))}typeof r.profileIcon=="string"&&r.profileIcon&&(t(`Perfil ${n+1}: icono del perfil…`),await ve(n,20,Pr(r.profileIcon)))}ka(),B(),await fe()}async function Rl(e){try{e("Leyendo el teclado…");const t=await Sl(e);e("Guardando archivo…");const a=await window.orby.saveBackup(t);if(a.canceled)return;if(!a.ok)throw new Error(a.error||"error desconocido");g("Copia guardada correctamente")}catch(t){g(`No se pudo hacer la copia: ${t.message}`,"error")}finally{e(null)}}async function Nl(e){try{const t=await window.orby.loadBackup();if(t.canceled)return;if(!t.ok)throw new Error(t.error||"error desconocido");const a=t.data?.profiles?.length??0;if(!confirm(`Se sobrescribirán los ${a} perfiles del teclado con la copia del ${new Date(t.data.savedAt).toLocaleString("es-ES")}.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`))return;e("Restaurando…"),await Tl(t.data,e),g("Copia restaurada; pulsa Guardar en Flash para fijarla")}catch(t){g(`No se pudo restaurar: ${t.message}`,"error")}finally{e(null)}}let Zn=!1,cn=null;const Ol=400,Di=2;async function ql(){let e=null;try{e=(await window.orby.getConfig())?.deviceMirror??null}catch{e=null}return Zn=!0,!e?.snapshot?.profiles?.length||s.profiles.length?!1:(_l(e),console.debug(`[mirror] copia del PC cargada: ${s.profiles.length} perfiles,`,`${Object.keys(e.icons||{}).length} iconos`),te(),!0)}function _l(e){const t=e.snapshot;s.profiles=t.profiles.map((n,r)=>zl(n,r)),s.activeProfileIdx=Math.min(t.activeProfile||0,s.profiles.length-1),Number.isInteger(t.timeout)&&(s.timeout=t.timeout),Number.isInteger(t.maxProfiles)&&(s.maxProfiles=t.maxProfiles);const a=s.profiles[s.activeProfileIdx];if(s.pageIdx=0,s.pageCount=a?.pageCount||1,s.maxPages=Math.max(...s.profiles.map(n=>n.maxPages||1),1),tl(e.icons),(e.iconsVersion||0)<Di)for(const n of s.profiles)D(n)>1&&Ci(n.idx)}function zl(e,t){const a={idx:t,name:e.name||"",pageCount:e.pageCount||1,maxPages:e.maxPages||1,pageIdx:0,pages:[]},n=e.pages&&e.pages.length?e.pages:[{labels:e.labels,keys:e.keys,rotary:e.rotary,scroll:e.scroll,oledMask:e.oledMask}];return a.pages=n.map(r=>{const i=Za();return ta(i.labels,r?.labels,""),ta(i.keys,r?.keys,null),ta(i.rotary,r?.rotary,null),ta(i.scroll,r?.scroll,null),i.oledMask=r?.oledMask||0,i}),a.pageCount=Math.max(1,Math.min(a.pageCount,a.pages.length)),fi(a)}function ta(e,t,a){if(Array.isArray(t))for(let n=0;n<e.length&&n<t.length;n++)t[n]===void 0||t[n]===null||(e[n]=a===""?String(t[n]):t[n])}function Bl(){it(()=>Cr()),Fn(()=>Cr())}function Cr(){!Zn||s.syncing||!s.profiles.length||s.connected&&jl()}function jl(){clearTimeout(cn),cn=setTimeout(ra,Ol)}let Sr=null;function ra(){if(clearTimeout(cn),!Zn||!s.profiles.length)return;const e=Il(),t=el(),a=JSON.stringify({snapshot:{...e,savedAt:null},icons:t});if(a!==Sr)try{window.orby.setConfig({deviceMirror:{savedAt:new Date().toISOString(),snapshot:e,icons:t,iconsVersion:Di}}),Sr=a}catch(n){console.error("No se pudo guardar el espejo local:",n)}}let Fe=null,da=null,ua=!1;const Dl=7e3;function Hl(){Fe=document.getElementById("app-splash"),da=document.getElementById("splash-msg"),Fe&&setTimeout(Hi,Dl)}function Vl(){return!!Fe&&!ua}function pa(e){da&&!ua&&(da.textContent=e)}function Hi(){ua||!Fe||(ua=!0,Fe.classList.add("gone"),setTimeout(()=>{Fe?.remove(),Fe=null,da=null},320))}const dn=new Set,re={status:"idle",version:"",newVersion:null,percent:0,error:null};function Ba(e){e&&(Object.assign(re,e),dn.forEach(t=>t(re)))}function Vi(e){return dn.add(e),()=>dn.delete(e)}async function Fl(){if(!window.orby?.updater){Ba({status:"dev"});return}window.orby.updater.onState(Ba),Wl(),Ba(await window.orby.updater.get())}function Wl(){let e=null;const t=()=>{s.dirty!==e&&(e=s.dirty,window.orby.updater.ocupado(s.dirty))};it(t),t()}function Ul(){return window.orby?.updater?.check?.()??null}function Fi(){return window.orby?.updater?.install?.()??!1}function Gl(){switch(re.status){case"dev":return"No disponible en modo desarrollo";case"checking":return"Comprobando…";case"downloading":return`Descargando ${re.newVersion} (${re.percent}%)`;case"downloaded":return`${re.newVersion} lista: se instalará al guardar los cambios`;case"error":return`Error: ${re.error}`;default:return"Estás en la última versión"}}let pt=[],Wi=!1;const Ui=new Set;async function ia(){try{pt=await window.orby.plugins.list()}catch{pt=[]}Wi=!0;for(const e of Ui)e();return pt}const Kl=ia;function Jn(e){Ui.add(e),Wi&&e()}function Xl(){return pt}function er(){return pt.filter(e=>e.enabled&&!e.error)}function Je(e){return pt.find(t=>t.id===e)||null}function Ca(e,t){return(e?.actions||[]).filter(a=>a.targets.includes(t))}function Gi(e){return er().filter(t=>Ca(t,e).length>0)}function Sa(e,t){const a=Je(e),n=(a?.actions||[]).find(r=>r.op===t)||null;return{plugin:a,action:n}}function Ia(e){return e?.views||[]}function Yl(){return er().filter(e=>e.hasRead&&Ia(e).length>0)}function Ql(e,t){const a=Je(e),n=Ia(a).find(r=>r.op===t)||null;return{plugin:a,view:n}}async function Zl(e,t){try{return await window.orby.plugins.read(e,t)}catch{return{ok:!1}}}function Ki(e){if(!e)return"Sin asignar";const{plugin:t,action:a}=Sa(e.plugin,e.op);if(!t)return`Complemento «${e.plugin}» (no instalado)`;if(!a){const{view:n}=Ql(e.plugin,e.op);return n?`${t.name}: ${n.label.toLowerCase()} (pantalla)`:`${t.name}: acción desconocida`}if(a.value&&Number.isFinite(e.value))return`${a.label} (${e.value})`;if(a.targets.includes("turn")&&Number(e.value)){const n=Number(e.value)<0?"−":"+";return`${a.label} ${n}${Math.abs(Number(e.value))}`}return`${t.name}: ${a.label.toLowerCase()}`}const Ir="2.0",Et="4.6",tr={read:{since:"2.0",label:"leer la configuración del teclado"},profiles:{since:"3.0",label:"crear y borrar perfiles"},superDial:{since:"3.0",label:"mandos en la capa SUPER"},perProfileWheel:{since:"3.0",label:"rueda ajustable por perfil"},pages:{since:"4.0",label:"páginas dentro de un perfil",flag:"maxpages"},macros:{since:"4.0",label:"secuencias reproducidas por el teclado",flag:"macros"},hash:{since:"4.1",label:"sincronización por huella y precarga de iconos",flag:"hash"},bootsel:{since:"4.2",label:"actualizar el firmware desde la app sin tocar el botón",flag:"bootsel"},profileIcon:{since:"4.4",label:"icono propio para cada perfil",flag:"picon"},hostApp:{since:"4.5",label:"tachar en las pantallas las teclas que necesitan la app",flag:"hostapp"}};function un(e,t){const a=i=>String(i??"").split(".").map(o=>parseInt(o,10)||0),n=a(e),r=a(t);for(let i=0;i<Math.max(n.length,r.length);i++){const o=(n[i]||0)-(r[i]||0);if(o)return o<0?-1:1}return 0}function Xi(e){return e?.fw||"0.0"}function lt(e,t){const a=tr[t];if(!a)return!1;if(a.flag&&e&&a.flag in e){const n=e[a.flag];return n!=="0"&&n!==""&&n!=null}return un(Xi(e),a.since)>=0}function Jl(e){return Object.entries(tr).filter(([t])=>!lt(e,t)).map(([,t])=>t.label)}function Yi(e){const t=Xi(e);if(un(t,Ir)<0)return{level:"blocked",fw:t,title:`Firmware ${t}: demasiado antiguo`,detail:`Esta versión de OrbyGUI necesita el firmware ${Ir} o posterior. Flashea el ${Et} para usar el editor.`};if(un(t,Et)>0)return{level:"app-vieja",fw:t,title:`Firmware ${t}: más nuevo que esta app`,detail:`OrbyGUI conoce hasta el ${Et}. Lo que ya funciona sigue funcionando, pero las funciones nuevas del teclado no se ven hasta que actualices la app.`};const a=Jl(e);return a.length?{level:"recortado",fw:t,title:`Firmware ${t}: faltan funciones`,detail:`Sin flashear el ${Et} no tendrás: ${a.join(", ")}.`,missing:a}:{level:"ok",fw:t,title:`Firmware ${t}`,detail:"Al día."}}const pn=new Set,G={status:"idle",current:null,latest:null,available:!1,percent:0,manual:!1,error:null};function fa(e){e&&(Object.assign(G,e),pn.forEach(t=>t(G)))}function ec(e){return pn.add(e),()=>pn.delete(e)}function La(){return!!window.orby?.firmware}async function tc(){La()&&(window.orby.firmware.onState(fa),fa(await window.orby.firmware.get()))}async function Qi(){if(!La())return null;const e=await window.orby.firmware.check({maxFw:Et,currentFw:s.deviceInfo?.fw??null});return fa(e),e}async function ac(){if(!La())return null;const e=await window.orby.firmware.update({viaBootsel:lt(s.deviceInfo,"bootsel")});return fa(e),e}function nc(){return window.orby?.firmware?.cancel?.()??null}function rc(){switch(G.status){case"checking":return"Comprobando…";case"downloading":return`Descargando ${G.latest?.version??""} (${G.percent}%)`;case"bootsel":return G.manual?"Desenchufa el teclado, enchúfalo con BOOTSEL pulsado y suéltalo":"Reiniciando el teclado en modo de actualización…";case"flashing":return"Copiando el firmware… no desconectes el teclado";case"done":return"Firmware actualizado. El teclado se está reiniciando";case"error":return`Error: ${G.error}`;default:return G.latest?G.available?`Disponible la ${G.latest.version}`:"El teclado está al día":"Sin comprobar"}}function Lr(){return["checking","downloading","bootsel","flashing"].includes(G.status)}const Aa=[{bit:1,label:"Ctrl",short:"Ctrl"},{bit:2,label:"Shift",short:"Shift"},{bit:4,label:"Alt",short:"Alt"},{bit:8,label:"Win",short:"Win"},{bit:16,label:"Ctrl der",short:"RCtrl"},{bit:32,label:"Shift der",short:"RShift"},{bit:64,label:"AltGr",short:"AltGr"},{bit:128,label:"Win der",short:"RWin"}],gt=254,et=253,tt=252,M=251,Yt=[{index:7,label:"Subir volumen"},{index:8,label:"Bajar volumen"},{index:9,label:"Silenciar"},{index:10,label:"Subir brillo"},{index:11,label:"Bajar brillo"},{index:3,label:"Reproducir / Pausa"},{index:4,label:"Detener"},{index:5,label:"Pista siguiente"},{index:6,label:"Pista anterior"},{index:12,label:"Abrir navegador"},{index:1,label:"Abrir explorador"},{index:2,label:"Abrir calculadora"},{index:13,label:"Abrir correo"},{index:14,label:"Buscar"},{index:15,label:"Acercar (zoom +)"},{index:16,label:"Alejar (zoom −)"}],Ta=[{id:"volumen",label:"Volumen",up:7,down:8},{id:"brillo",label:"Brillo",up:10,down:11},{id:"zoom",label:"Zoom",up:15,down:16}],ic=new Set(Ta.flatMap(e=>[e.up,e.down])),oc=[...Ta.map(e=>({pairId:e.id,label:e.label,up:e.up,down:e.down})),...Yt.filter(e=>!ic.has(e.index))];function ar(e){return Ta.find(t=>t.up===e||t.down===e)||null}const C={NONE:0,CONSUMER:1,KEY:2,SCROLL_V:3,SCROLL_H:4,ZOOM:5},R={ENC1_CW:0,ENC1_CCW:1,ENC1_CLICK:2,ENC2_CW:3,ENC2_CCW:4,ENC2_CLICK:5,WHEEL_CW:6,WHEEL_CCW:7};function nr(e){return e===C.SCROLL_V||e===C.SCROLL_H||e===C.ZOOM}function sc(e){if(!e)return"Sin asignar";switch(e.type){case C.SCROLL_V:return"Desplazar vertical";case C.SCROLL_H:return"Desplazar horizontal";case C.ZOOM:return"Zoom (Ctrl + rueda)";case C.CONSUMER:{const t=Yt.find(a=>a.index===e.keycode);return t?t.label:`Multimedia ${e.keycode}`}case C.KEY:return _t(e.modifier,e.keycode);default:return"Sin asignar"}}function Mt(e,t,a){const n=[];for(let r=e;r<=t;r++)n.push(a(r));return n}const Zi=[{name:"Letras",keys:Mt(0,25,e=>({code:4+e,label:String.fromCharCode(65+e)}))},{name:"Números",keys:[...Mt(0,8,e=>({code:30+e,label:String(e+1)})),{code:39,label:"0"}]},{name:"Función",keys:[...Mt(0,11,e=>({code:58+e,label:`F${e+1}`})),...Mt(0,11,e=>({code:104+e,label:`F${e+13}`}))]},{name:"Edición",keys:[{code:40,label:"Enter"},{code:41,label:"Esc"},{code:42,label:"Retroceso"},{code:43,label:"Tab"},{code:44,label:"Espacio"},{code:73,label:"Insert"},{code:76,label:"Supr"},{code:74,label:"Inicio"},{code:77,label:"Fin"},{code:75,label:"Re Pág"},{code:78,label:"Av Pág"},{code:70,label:"Impr Pant"}]},{name:"Flechas",keys:[{code:80,label:"←"},{code:79,label:"→"},{code:82,label:"↑"},{code:81,label:"↓"}]},{name:"Símbolos",keys:[{code:45,label:"-"},{code:46,label:"="},{code:47,label:"["},{code:48,label:"]"},{code:49,label:"\\"},{code:51,label:";"},{code:52,label:"'"},{code:53,label:"`"},{code:54,label:","},{code:55,label:"."},{code:56,label:"/"}]},{name:"Teclado numérico",keys:[{code:84,label:"Num /"},{code:85,label:"Num *"},{code:86,label:"Num -"},{code:87,label:"Num +"},{code:88,label:"Num Enter"},...Mt(1,9,e=>({code:88+e,label:`Num ${e}`})),{code:98,label:"Num 0"}]}],Ji=new Map;for(const e of Zi)for(const t of e.keys)Ji.set(t.code,t.label);function lc(e){return Ji.get(e)||(e?`0x${e.toString(16).toUpperCase()}`:"—")}function _t(e,t){if(e===gt){const n=Yt.find(r=>r.index===t);return n?n.label:`Multimedia ${t}`}if(e===et)return`Ir a la página ${t}`;if(e===tt)return"Estado de páginas";if(e===M)return`Secuencia ${t}`;if(!e&&!t)return"Sin asignar";const a=Aa.filter(n=>e&n.bit).map(n=>n.short);return t&&a.push(lc(t)),a.join(" + ")}const cc=(()=>{const e=new Map;for(let a=0;a<26;a++)e.set(`Key${String.fromCharCode(65+a)}`,4+a);for(let a=1;a<=9;a++)e.set(`Digit${a}`,30+a-1);e.set("Digit0",39);for(let a=1;a<=24;a++)e.set(`F${a}`,a<=12?58+a-1:104+a-13);const t={Enter:40,Escape:41,Backspace:42,Tab:43,Space:44,Minus:45,Equal:46,BracketLeft:47,BracketRight:48,Backslash:49,Semicolon:51,Quote:52,Backquote:53,Comma:54,Period:55,Slash:56,CapsLock:57,PrintScreen:70,ScrollLock:71,Pause:72,Insert:73,Home:74,PageUp:75,Delete:76,End:77,PageDown:78,ArrowRight:79,ArrowLeft:80,ArrowDown:81,ArrowUp:82,NumLock:83,NumpadDivide:84,NumpadMultiply:85,NumpadSubtract:86,NumpadAdd:87,NumpadEnter:88,Numpad0:98};for(const[a,n]of Object.entries(t))e.set(a,n);for(let a=1;a<=9;a++)e.set(`Numpad${a}`,88+a);return e})();function Ar(e){const t=cc.get(e.code)||0;let a=0;return e.ctrlKey&&(a|=1),e.shiftKey&&(a|=2),e.altKey&&(a|=4),e.metaKey&&(a|=8),{modifier:a,keycode:t}}let ge=[];function dc(){return ge}function uc(e){ge=ge.filter(t=>t.id!==e),E(e)}async function yt(){try{ge=(await window.orby.getConfig())?.macros||[]}catch{ge=[]}}function E(e){window.orby.setConfig({macros:ge}),e!==void 0&&ro(e)}function I(e){return ge.find(t=>t.id===e)}function oe(e){let t=I(e);return t||(t={id:e,actions:[]},ge.push(t),E(e)),t}function me(){const e=new Set(ge.map(a=>a.id));let t=1;for(;e.has(t);)t++;return t}const eo={delay:1,hotkey:2,mouse_move:3,mouse_click:4},to=48,pc={left:0,middle:1,right:2},fc=64;function rr(){const e=parseInt(s.deviceInfo?.maxmacros??"",10);return Number.isFinite(e)&&e>0?e:fc}const ao={sleep:"Suspender",hibernate:"Hibernar",restart:"Reiniciar",shutdown:"Apagar",lock:"Bloquear pantalla",logoff:"Cerrar sesión"},no=20;function zt(e){const t=e?.actions||[];return t.length>0&&t.length<=to&&e.id<rr()&&t.every(a=>a.type in eo)}function hc(e){return e.actions.map(t=>{const a=eo[t.type];let n=0,r=0;t.type==="delay"?n=Math.max(0,Math.min(32767,Math.round(t.ms)||0)):t.type==="hotkey"?(n=t.modifier||0,r=t.keycode||0):t.type==="mouse_move"?(n=t.dx||0,r=t.dy||0):t.type==="mouse_click"&&(n=pc[t.button]??0);const o=t.type==="hotkey"||t.type==="mouse_click"?Math.max(1,Math.min(255,t.count||1)):1,c=o>1?Math.max(0,Math.min(32767,Math.round(t.gap??no))):0;return{type:a,a:n,b:r,repeat:o,gap:c}})}async function Tr(e,t){let a;try{a=await bs(e)}catch{return!1}return!a||a.length!==t.length?!1:t.every((n,r)=>{const i=a[r];return i&&i.type===n.type&&i.a===n.a&&i.b===n.b&&i.repeat===n.repeat&&i.gap===n.gap})}async function ro(e){if(!s.connected||!lt(s.deviceInfo,"macros")||e>=rr())return;const t=I(e);if(!t||!zt(t)){if(await Tr(e,[]))return;try{await ys(e)}catch{}B();return}const a=hc(t);if(!await Tr(e,a))try{for(let n=0;n<a.length;n++){const r=a[n];await ms(e,n,r.type,r.a,r.b,r.repeat,r.gap)}await gs(e,a.length),B()}catch{}}async function io(){if(!lt(s.deviceInfo,"macros"))return;await yt();const e=await mc();for(const n of ge)!zt(n)&&e&&!e.includes(n.id)||await ro(n.id);const t=rr(),a=ge.filter(n=>zt(n)&&n.id<t).map(n=>n.id);window.orby.setConfig({macrosOnDevice:a})}async function mc(){try{const e=await window.orby.getConfig();return Array.isArray(e?.macrosOnDevice)?e.macrosOnDevice:null}catch{return null}}function ha(e){const t=I(e)?.actions||[];return t.length===1&&t[0].type==="open_app"}function ma(e){const t=I(e)?.actions||[];return t.length===1&&t[0].type==="text"}function Qt(e){const t=I(e)?.actions||[];return t.length===1&&t[0].type==="system_power"}function wt(e){const t=I(e)?.actions||[];return t.length===1&&t[0].type==="plugin"}function je(e){return wt(e)?I(e).actions[0]:null}function Bt(e){return e?.type===C.KEY&&e.modifier===M&&wt(e.keycode)?Ki(I(e.keycode).actions[0]):sc(e)}function oo(e){return I(e)?.kind==="recording"}function ir(e){return I(e)?.kind==="recording-reset"}function At(e){return oo(e)||ir(e)}function gc(e){if(e.modifier!==M)return _t(e.modifier,e.keycode);if(ir(e.keycode))return"Borrar grabación";if(oo(e.keycode))return I(e.keycode).events?.length?"Reproducir grabación":"Grabar operación";if(ha(e.keycode)){const t=I(e.keycode).actions[0].target||"";return t?`Abrir ${t.split(/[\\/]/).pop()}`:"Abrir…"}if(ma(e.keycode)){const t=(I(e.keycode).actions[0].text||"").replace(/\s+/g," ").trim();return t?`Escribir "${t.length>14?`${t.slice(0,14)}…`:t}"`:"Escribir texto"}return Qt(e.keycode)?ao[I(e.keycode).actions[0].mode]||"Energía":wt(e.keycode)?Ki(I(e.keycode).actions[0]):_t(e.modifier,e.keycode)}function so(e){if(!e||e.modifier!==M)return!1;const t=I(e.keycode);return t?!zt(t):!1}function fn(e){return!e||e.type!==C.KEY?!1:so(e)}const yc=2e3,Rr=new Map;function bc(e){let t=Rr.get(e);if(!t){const a=f(e,96).replace("currentColor","#fff");t=Pi(a,{size:96}),Rr.set(e,t)}return t}function vc(e,t,a,n){const r=ot(e),i=n/Math.max(1,r.height);return{x:t-r.x*i,y:a-r.y*i,scale:i,threshold:128,blur:0,dither:!1,invert:"none"}}async function Nr(e,t,a,n,r){try{const i=await bc(t),o=Kt(i,vc(i,a,n,r));return Hn(e,o,"merge")}catch(i){return console.error(`[live-oled] icono "${t}":`,i),e}}const ja=4,Da=w-4,Ha=26,Va=34;function $c(e,t){for(let o=ja;o<=Da;o++)ne(e,o,Ha,1),ne(e,o,Va,1);for(let o=Ha;o<=Va;o++)ne(e,ja,o,1),ne(e,Da,o,1);const a=Math.max(0,Math.min(100,Number(t)||0))/100,n=ja+2,r=Da-2,i=Math.round(n+(r-n)*a);for(let o=n;o<=i;o++)for(let c=Ha+2;c<=Va-2;c++)ne(e,o,c,1)}async function xc(e,t,a){let n=Gt();return n=await Nr(n,e,4,3,15),n=await Nr(n,t,w-19,3,15),$c(n,a),n}function wc(){if(!s.connected)return[];const e=s.activeProfileIdx,a=s.profiles[e]?.pages?.[s.pageIdx];if(!a)return[];const n=s.superActive?"super":"normal",r=[];for(let i=0;i<ze.length;i++){if(!ze[i])continue;const o=a.keys[le(i,n)];if(!o||o.modifier!==M)continue;const c=I(o.keycode),u=c?.actions?.length===1&&c.actions[0].type==="plugin"?c.actions[0]:null;if(!u)continue;const p=Je(u.plugin);if(!p?.enabled||!p.hasRead)continue;const h=Ia(p).find(v=>v.op===u.op);h&&r.push({key:`${e}:${s.pageIdx}:${F(i,n)}`,profileIdx:e,pageIdx:s.pageIdx,oledSlot:F(i,n),pluginId:p.id,pluginIcon:p.icon,op:u.op,viewIcon:h.icon})}return r}let ga=null;const Tt=new Map;async function Mc(e){if(!(e.profileIdx!==s.activeProfileIdx||e.pageIdx!==s.pageIdx))try{const t=ye(e.profileIdx,e.oledSlot,e.pageIdx);t?await ve(e.profileIdx,e.oledSlot,t):await Wt(e.profileIdx,e.oledSlot)}catch{}}async function kc(e){const t=await Zl(e.pluginId,e.op);if(!t.ok){console.warn(`[live-oled] "${e.op}" de "${e.pluginId}" no respondió`);return}try{const a=await xc(e.pluginIcon,e.viewIcon,t.value);await ve(e.profileIdx,e.oledSlot,a)}catch(a){console.error(`[live-oled] no se pudo subir la tecla ${e.oledSlot}:`,a)}}let Or="";async function hn(){const e=wc(),t=new Set(e.map(n=>n.key)),a=e.map(n=>n.key).join(",");a!==Or&&(Or=a,console.debug("[live-oled] teclas visoras activas:",e));for(const[n,r]of Tt)t.has(n)||(Tt.delete(n),Mc(r));for(const n of e)Tt.set(n.key,n);await Promise.all(e.map(kc))}function qr(){ga||(ga=setInterval(hn,yc),hn())}function _r(){clearInterval(ga),ga=null}function Ec(){V("plugins")&&(yt(),it(()=>s.connected?qr():(_r(),Tt.clear())),Jn(()=>{s.connected&&hn()}),W("connected",()=>{yt(),qr()}),W("disconnected",()=>{_r(),Tt.clear()}))}let zr=null,Fa=0;function lo(){const e=document.getElementById("keyboard-visualizer");let t=`
    <div class="orby-board">
      <div class="board-keys">`;for(let a=1;a<=12;a++){const n=ze[a-1]!==0;t+=`
      <div class="hw-key ${n?"":"no-screen"}" id="hw-key-${a}">
        ${n?`<div class="oled-screen" id="hw-oled-${a}">--</div>`:`<span class="hw-key-role">${a===ht?"SUPER":"MENU"}</span>`}
      </div>`}t+=`
      </div>
      <div class="board-controls">
        <div class="encoder-row">
          <div class="hw-encoder" id="hw-enc-1"><span class="hw-tag">ENC 1</span></div>
          <div class="hw-encoder" id="hw-enc-2"><span class="hw-tag">ENC 2</span></div>
        </div>
        <div class="hw-wheel" id="hw-scroll">
          <div class="hw-wheel-face">
            ${Xn("hw-wheel-needle")}
            <div class="hw-wheel-hub"></div>
          </div>
          <span class="hw-tag">RUEDA</span>
        </div>
      </div>
    </div>`,e.innerHTML=t,W("telemetry",Pc),Ii(co),Fn(Ke),Ke()}function co(e){const t=document.getElementById("hw-wheel-needle");t&&(t.style.transform=`rotate(${e}deg)`)}function Ra(){const e=document.querySelector(".hw-wheel-face");e&&(e.querySelector(".dial-marker")?.remove(),e.insertAdjacentHTML("afterbegin",Xn("hw-wheel-needle")),co(Gn()))}function Pc(e){if(e.startsWith("KEY_EV:")){const[,t,a]=e.split(":"),n=parseInt(t,10),r=a==="1";n===ht&&(s.superActive=r,te(),Ke()),r?(Wa(`hw-key-${n}`),Pt(`Tecla ${n}`,Cc(n))):document.getElementById(`hw-key-${n}`)?.classList.remove("active")}else if(e.startsWith("ENC:")){const[,t,a]=e.split(":");Wa(`hw-enc-${t}`);const n=parseInt(a,10);Pt(`Encoder ${t}`,`${n>0?"+":""}${n}`)}else if(e.startsWith("ENC_SW:")){const[,t,a]=e.split(":");a==="1"&&(Wa(`hw-enc-${t}`),Pt(`Encoder ${t}`,"Click"))}else e.startsWith("WHEEL:")?Sc(parseInt(e.slice(6),10)):e.startsWith("MODE:")?(s.deviceMode=e.split(":")[1],te(),Ke(),Pt("Modo",s.deviceMode)):e.startsWith("PROFILE:OK:")&&(s.activeProfileIdx=parseInt(e.split(":")[2],10),te(),Ke())}function Cc(e){const t=$t();if(!t)return"";const a=s.superActive?"super":"normal";return vi(t,e-1,a)||""}function Sc(e){if(!Number.isFinite(e))return;Fa+=e;const t=document.getElementById("hw-scroll");if(!t)return;t.classList.add("active");const a=wi().detentsPerRev||60,n=Fa*a/4096;Pt("Scroll",`${n>=0?"+":""}${n.toFixed(2)} clics`),clearTimeout(zr),zr=setTimeout(()=>{t.classList.remove("active"),Fa=0},600)}function Wa(e){const t=document.getElementById(e);t&&(t.classList.add("active"),setTimeout(()=>t.classList.remove("active"),160))}function Pt(e,t){const a=document.getElementById("live-event-feed");a&&(a.innerHTML=`
    <div class="big-event">
      <span class="big-event-title">${e}</span>
      ${t?`<span class="big-event-detail">${t}</span>`:""}
    </div>`)}function Ke(){const e=$t(),t=s.superActive?"super":"normal";document.getElementById("lbl-active-profile").textContent=e?e.name:"--",document.getElementById("lbl-active-mode").textContent=s.deviceMode;const a=document.getElementById("row-active-page");if(a){const i=e?D(e):1;a.hidden=!rt(),document.getElementById("lbl-active-page").textContent=`${Math.min(s.pageIdx,i-1)+1} de ${i}`}const n=document.getElementById("lbl-super-state");n.textContent=s.superActive?"Activa":"Inactiva",n.classList.toggle("is-on",s.superActive);const r=document.getElementById("lbl-scroll-mode");if(r){const i=wi();r.textContent=`${i.detentsPerRev} clics/vuelta · ${i.hires?"suave":"clásico"}`,r.classList.toggle("is-on",i.hires)}for(let i=1;i<=12;i++){const o=document.getElementById(`hw-oled-${i}`);if(!o)continue;const c=F(i-1,t);(e&&c>=0?ye(s.activeProfileIdx,c):null)?o.innerHTML=`<canvas class="okey-canvas" data-bmp="${ue(s.activeProfileIdx,c)}"></canvas>`:o.textContent=e&&vi(e,i-1,t)||"--"}Xt(document.getElementById("keyboard-visualizer"))}const Ic=Object.freeze(Object.defineProperty({__proto__:null,init:lo,refreshMarker:Ra,render:Ke},Symbol.toStringTag,{value:"Module"})),uo=[{name:"Izquierdo",icon:"reset",parts:[{slot:R.ENC1_CW,label:"Giro horario",short:"↻"},{slot:R.ENC1_CCW,label:"Giro antihorario",short:"↺"},{slot:R.ENC1_CLICK,label:"Pulsación",short:"⏺",discrete:!0}]},{name:"Derecho",icon:"reset",parts:[{slot:R.ENC2_CW,label:"Giro horario",short:"↻"},{slot:R.ENC2_CCW,label:"Giro antihorario",short:"↺"},{slot:R.ENC2_CLICK,label:"Pulsación",short:"⏺",discrete:!0}]}],Lc={name:"Rueda de scroll",icon:"wheel",parts:[{slot:R.WHEEL_CW,label:"Hacia abajo",short:"↓"},{slot:R.WHEEL_CCW,label:"Hacia arriba",short:"↑"}]},Ac=[{type:C.NONE,label:"Nada"},{type:C.CONSUMER,label:"Multimedia / sistema"},{type:C.KEY,label:"Atajo de teclado"},{type:C.SCROLL_V,label:"Desplazar vertical",turnOnly:!0},{type:C.SCROLL_H,label:"Desplazar horizontal",turnOnly:!0},{type:C.ZOOM,label:"Zoom (Ctrl + rueda)",turnOnly:!0}],Tc=[{value:12,name:"Preciso",desc:"Timeline, PCB, edición fina"},{value:30,name:"Suave",desc:"Lectura y navegación lenta"},{value:60,name:"Estándar",desc:"Equivalente a rueda de ratón"},{value:120,name:"Rápido",desc:"Documentos y logs largos"}],po=50,Rc=360,bt=new Set([R.ENC1_CCW,R.ENC2_CCW,R.WHEEL_CW]),Zt={[R.ENC1_CW]:R.ENC1_CCW,[R.ENC1_CCW]:R.ENC1_CW,[R.ENC2_CW]:R.ENC2_CCW,[R.ENC2_CCW]:R.ENC2_CW,[R.WHEEL_CW]:R.WHEEL_CCW,[R.WHEEL_CCW]:R.WHEEL_CW};let ya={render:()=>{},renderKeyGrid:()=>{}};function Nc(e){ya={...ya,...e}}function T(){ya.render()}function Br(){ya.renderKeyGrid()}const d={editingProfile:0,layer:"normal",variantId:null,selected:null,tab:"shortcut",capturing:!1,busy:!1};function ba(){const e=s.profiles[d.editingProfile];return e&&e.pageIdx||0}function U(){const e=d.variantId?st(d.variantId):null;return e&&e.profile===d.editingProfile&&e.page===ba()?e:null}function Re(){return d.selected?.kind==="key"?d.selected.index:null}function xe(){return d.selected?.kind==="rotary"?d.selected.slot:null}function fo(e){for(const t of[...uo,Lc]){const a=t.parts.find(n=>n.slot===e);if(a)return{group:t,part:a}}return null}function O(){return s.profiles[d.editingProfile]||null}function Ne(){return document.getElementById("view-profiles")?.classList.contains("active")}function J(){const e=O(),t=Re();return!e||t===null?{modifier:0,keycode:0}:zi(e,U(),t,d.layer)}function _e(){const e=O(),t=xe();return!e||t===null?{type:0,modifier:0,keycode:0}:Pe(e,U(),t,d.layer)}function or(){return Nn(O(),d.layer)}function A(){const e=d.selected?.kind==="rotary"?_e():J();return e.modifier===M?e.keycode:null}const ee={open:!1,plugin:null,op:null,mode:null,value:0};async function ho(e,t){if(!Y())return;const a=O();if(!a)return;const n=U();if(n){if(Yn(n.id,"labels",e,t),Br(),Te(n.id))try{await Pa(n.id,"labels",e,t)}catch{g("El teclado no confirmó la etiqueta","error")}return}a.labels[e]=t;try{await Vt(d.editingProfile,e,t),B(),await Ea("labels",e),Br()}catch{g("El teclado no confirmó la etiqueta","error")}}async function K(e,t){if(!Y())return;const a=O(),n=Re();if(!a||n===null)return;const r=le(n,d.layer),i=U();if(i){if(Yn(i.id,"keys",r,{modifier:e,keycode:t}),T(),Te(i.id))try{await Pa(i.id,"keys",r,{modifier:e,keycode:t})}catch{g("No se pudo escribir la asignación","error")}return}a.keys[r]={modifier:e,keycode:t},T();try{await nt(d.editingProfile,r,e,t),B(),await Ea("keys",r)}catch{g("No se pudo escribir la asignación","error")}}async function Oc(){if(!Y())return;const e=O(),t=Re();if(!e||t===null)return;const a=F(t,d.layer);if(!(a<0)){await ho(a,"");try{const n=new Uint8Array(Rc);await ve(d.editingProfile,a,n),Be(d.editingProfile,a,n),e.oledMask|=1<<a,B(),T()}catch{g("No se pudo apagar la pantalla de la tecla","error")}}}let de=null;async function qc(){const e=O(),t=Re();if(!e||t===null)return;const a=J(),n=F(t,d.layer),r=n>=0?Bi(e,U(),t,d.layer):"",i=n>=0?ye(d.editingProfile,n):null;let o=null;if(a.modifier===M){const c=I(a.keycode);c&&(o=JSON.parse(JSON.stringify(c.actions||[])))}de={action:{modifier:a.modifier,keycode:a.keycode},label:r,icon:i?Uint8Array.from(i):null,macroActions:o},g("Tecla copiada"),T()}async function _c(){if(!de||!Y())return;const e=O(),t=Re();if(!e||t===null)return;let{modifier:a,keycode:n}=de.action;if(a===M&&de.macroActions){const i=me(),o=oe(i);o.actions=JSON.parse(JSON.stringify(de.macroActions)),E(i),n=i}await K(a,n);const r=F(t,d.layer);if(r<0){g("Tecla pegada");return}await ho(r,de.label||"");try{de.icon?(await ve(d.editingProfile,r,de.icon),Be(d.editingProfile,r,Uint8Array.from(de.icon)),e.oledMask|=1<<r,B()):ye(d.editingProfile,r)&&(await Wt(d.editingProfile,r),Be(d.editingProfile,r,null),e.oledMask&=~(1<<r),B()),T()}catch{g("No se pudo pegar el icono","error")}g("Tecla pegada")}async function mo(e){const t=O();if(!t)return;const a=U();if(a){for(const[n,r]of e)Yn(a.id,"rotary",n,r);if(T(),Te(a.id))try{for(const[n,r]of e)await Pa(a.id,"rotary",n,r)}catch{g("No se pudo escribir la acción del mando","error")}return}for(const[n,r]of e)t.rotary[n]=r;T();try{for(const[n,r]of e)await Ft(d.editingProfile,n,r.type,r.modifier,r.keycode),await Ea("rotary",n);B()}catch{g("No se pudo escribir la acción del mando","error")}}async function Xe(e){if(!Y())return;const t=O(),a=xe();if(!t||a===null)return;const n=pe(a,d.layer),r=Zt[a],i=r!==void 0&&nr(e.type),o=[[n,e]];i&&o.push([pe(r,d.layer),{...e}]),await mo(o)}async function sr(e,t,a,n){Y()&&O()&&await mo([[pe(e,d.layer),t],[pe(a,d.layer),n]])}const jr=[{id:"once",label:"Una vez",desc:"Reproduce la operación de principio a fin y para."},{id:"loop",label:"En bucle",desc:"Repite sin parar hasta que vuelvas a pulsar la tecla."},{id:"hold",label:"Mientras se pulsa",desc:"Repite mientras mantengas la tecla, y para al soltarla."}],zc=[1,2,3,5],go=3;let ft={id:null,phase:"idle"},vt=null,lr=null;const yo="RECORD",bo="PLAY",vo="RESET";function ct(e){const t=e===null?null:I(e);return t?.kind==="recording"?t:t?.kind==="recording-reset"&&I(t.target)||null}function Bc(e){const t=Jt(e);if(!t)return!1;const a=t.layer==="normal"?"super":"normal",n=s.profiles[t.profile]?.keys[le(t.key,a)];return n?.modifier===M&&I(n.keycode)?.target===e}function jc(e){const t=e?.events||[];return t.length?t[t.length-1].t:0}function Jt(e){for(let t=0;t<s.profiles.length;t++){const a=s.profiles[t];for(const n of["normal","super"])for(let r=0;r<12;r++){const i=a.keys[le(r,n)];if(i?.modifier===M&&i.keycode===e&&F(r,n)>=0)return{profile:t,key:r,layer:n}}}return null}function Dc(e){const t=Jt(e);return t?{profile:t.profile,slot:F(t.key,t.layer)}:null}async function Dr(e,t,a){if(t<0)return;const n=xo(a);await cr(async()=>{try{await ve(e,t,n),Be(e,t,Uint8Array.from(n));const r=s.profiles[e];r&&(r.oledMask|=1<<t),B()}catch{g("No se pudo escribir el icono de la tecla","error")}})}async function jt(e){const t=I(e),a=Jt(e);if(!t||!a)return;await Dr(a.profile,F(a.key,a.layer),t.events?.length?bo:yo);const n=a.layer==="normal"?"super":"normal",i=s.profiles[a.profile]?.keys[le(a.key,n)];i?.modifier===M&&I(i.keycode)?.target===e&&await Dr(a.profile,F(a.key,n),vo)}async function Hc(e){const t=O(),a=Re();if(!t||a===null||d.layer!=="normal")return;const n=le(a,"super"),r=me(),i=oe(r);i.kind="recording-reset",i.target=e,i.actions=[];const o=I(e);o&&(o.resetId=r),E(r),t.keys[n]={modifier:M,keycode:r};try{await nt(d.editingProfile,n,M,r),B(),await Ea("keys",n)}catch{g("No se pudo asignar la tecla de borrado en SUPER","error")}}async function $o(e){const t=I(e),a=Jt(e);if(!t||!a)return;const n=a.layer==="normal"?"super":"normal",r=le(a.key,n),i=s.profiles[a.profile],o=i?.keys[r];if(!(o?.modifier!==M||I(o.keycode)?.target!==e)){uc(o.keycode),i.keys[r]={modifier:0,keycode:0};try{await nt(a.profile,r,0,0),await Wt(a.profile,F(a.key,n)),Be(a.profile,F(a.key,n),null),i.oledMask&=~(1<<F(a.key,n)),B()}catch{g("No se pudo quitar la tecla de borrado de SUPER","error")}}}function xo(e){const t=_n(e,{fontSize:20,bold:!0,font:"Segoe UI Black"}),a=Dn(t),n=jn(t),r=Kt(t,{x:Math.round((w-n.width*a)/2),y:Math.round((S-n.height*a)/2),scale:a,threshold:128,blur:0,dither:!1,invert:"none"});return Ei(r),r}let Hr=Promise.resolve();function cr(e){const t=Hr.then(e,e);return Hr=t.catch(()=>{}),t}async function Ua(e,t){if(clearTimeout(lr),!s.connected)return;const a=Dc(e);if(!a)return;if(!vt){const r=ye(a.profile,a.slot);vt={...a,bytes:r?Uint8Array.from(r):null}}const n=xo(t);await cr(async()=>{try{await ve(a.profile,a.slot,n)}catch{}})}async function Vc(){clearTimeout(lr);const e=vt;vt=null,!(!e||!s.connected)&&await cr(async()=>{try{e.bytes?await ve(e.profile,e.slot,e.bytes):await Wt(e.profile,e.slot)}catch{}})}async function Fc({id:e,phase:t}){ft={id:e,phase:t},t==="recording"?await Ua(e,"REC"):t==="playing"?await Ua(e,"RUN"):t==="saved"||t==="empty"?(await yt(),await Ua(e,t==="saved"?"OK":"VACIO"),lr=setTimeout(async()=>{ft={id:null,phase:"idle"},vt=null,await jt(e),Ne()&&T()},1200)):t==="reset"?(await yt(),ft={id:null,phase:"idle"},vt=null,await jt(e),g("Grabación borrada: la tecla vuelve a estar lista para grabar")):await Vc(),Ne()&&T()}function Wc(e){const t=ct(A());t&&(t.mode=e,E(t.id),T())}function Uc(e){const t=ct(A());t&&(t.speed=Number(e)||go,E(t.id),T())}async function Gc(){const e=ct(A());!e||!e.events?.length||confirm(`Se borrará la operación grabada en esta tecla.

¿Continuar?`)&&(e.events=[],E(e.id),await jt(e.id),T())}async function Kc(){const e=ct(A());if(e){if(ft.phase!=="recording"&&e.events?.length){if(!confirm(`Se sustituirá la operación grabada en esta tecla.

¿Grabar de nuevo?`))return;e.events=[],await window.orby.setConfig({macros:dc()}),await jt(e.id),T()}window.orby.recorder.toggle(e.id)}}function Xc(e){const t=ct(e);if(!t)return'<p class="setting-desc">Preparando la grabación de esta tecla…</p>';if(ir(e)){const p=Jt(t.id),h=t.events||[];return`
      <div class="field">
        <span class="field-label">Borrar la grabación</span>
        <div class="rec-state ${h.length?"ok":"off"}">
          ${p?`Esta tecla borra lo grabado en la tecla ${p.key+1} de la capa normal.`:"Tecla de borrado."}
          ${h.length?` Ahora mismo hay ${h.length} eventos guardados.`:" Ahora mismo no hay nada que borrar."}
        </div>
      </div>
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-clear" ${h.length?"":"disabled"}>
          ${f("trash",16)} Borrar ahora
        </button>
      </div>
      <p class="setting-desc">
        Se creó sola al convertir la otra tecla en grabación. Para quitarla, cambia esa tecla a
        cualquier otra pestaña que no sea "Grabar".
      </p>`}const a=t.events||[],n=ft.id===t.id?ft.phase:"idle",r=n==="recording",i=n==="playing",o=t.mode||"once",c=t.speed||go,u=(jc(t)/1e3).toFixed(1);return`
    <div class="field">
      <span class="field-label">Operación grabada</span>
      <div class="rec-state ${r?"is-rec":a.length?"ok":"off"}">
        ${r?"Grabando… haz la operación y vuelve a pulsar la tecla (o el botón de abajo) para terminar.":a.length?`${a.length} eventos · ${u} s de ratón y teclado`:"Todavía no hay nada grabado en esta tecla."}
      </div>
    </div>

    <div class="inspector-actions">
      <button class="primary-btn ${r?"is-capturing":""}" data-act="rec-toggle">
        ${f(r?"square":"oled",16)}
        ${r?"Terminar grabación":a.length?"Grabar de nuevo":"Empezar a grabar"}
      </button>
      <button class="secondary-btn danger" data-act="rec-clear" ${a.length?"":"disabled"}>
        ${f("trash",16)} Borrar grabación
      </button>
    </div>

    <p class="setting-desc">
      Se captura lo que hagas con el ratón y el teclado del PC, con sus tiempos. Mientras grabas, la
      pantalla de la tecla pone <strong>REC</strong>; al terminar, <strong>OK</strong>. Para parar de
      grabar vale tanto la propia tecla como el botón de aquí arriba.
      ${a.length?'Con algo grabado, pulsar la tecla lo reproduce; para cambiarlo, usa "Grabar de nuevo".':""}
    </p>

    <div class="rec-state off">
      La pantalla de esta tecla pone <strong>${a.length?bo:yo}</strong>
      ${Bc(t.id)?`, y con SUPER pone <strong>${vo}</strong>: <strong>SUPER + esta tecla</strong>
           borra lo grabado y la deja lista para grabar otra vez.`:`. La tecla de borrado automática solo se monta al crear la grabación en la capa NORMAL:
           en SUPER no queda otra capa donde ponerla.`}
    </div>

    <div class="field mt-4">
      <span class="field-label">Al reproducir</span>
      <div class="chip-row">
        ${jr.map(p=>`
          <button class="chip ${o===p.id?"on":""}" data-act="rec-mode" data-mode="${p.id}">${p.label}</button>`).join("")}
      </div>
      <p class="setting-desc">${jr.find(p=>p.id===o)?.desc||""}</p>
    </div>

    <div class="field mt-4">
      <span class="field-label">Velocidad de reproducción</span>
      <div class="chip-row">
        ${zc.map(p=>`
          <button class="chip ${c===p?"on":""}" data-act="rec-speed" data-speed="${p}">${p}x</button>`).join("")}
      </div>
      <p class="setting-desc">Por defecto x3: reproduce tres veces más rápido que como se grabó.</p>
    </div>

    ${i?`
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-stop">${f("square",16)} Parar reproducción</button>
      </div>`:""}

    <p class="setting-desc">
      Esta acción la ejecuta siempre el PC (el teclado no puede saber dónde está el ratón), así que
      OrbyGUI tiene que estar abierto —vale con el icono de la bandeja— para que funcione.
    </p>`}async function Yc(){const e=me(),t=oe(e);t.kind="recording",t.mode="once",t.events=[],t.actions=[],E(e),await K(M,e),await Hc(e),await jt(e),T()}async function Na(){let e=await window.orby.foreground.current();if(e)return e;await window.orby.foreground.start();for(let t=0;t<8&&!e;t++)await new Promise(a=>setTimeout(a,300)),e=await window.orby.foreground.current();return e}function dr(e){return Zi.map(t=>`
    <optgroup label="${t.name}">
      ${t.keys.map(a=>`<option value="${a.code}" ${e===a.code?"selected":""}>${x(a.label)}</option>`).join("")}
    </optgroup>`).join("")}function x(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}function Qc(e,t,a,n){const{action:r}=Sa(e,t);r?.value&&(ee.open=!0,ee.plugin=e,ee.op=t,ee.mode=a,ee.value=Number.isFinite(n)?n:r.value.default,T())}function Zc(e){ee.value=e}function Jc(){ee.open=!1,T()}function ed(){const{plugin:e,op:t,mode:a,value:n}=ee;ee.open=!1,a==="rotary"?mn(e,t,n):Mo(e,t,n)}function td(e,t){return e.type!==t.type?!1:e.type==="hotkey"?e.modifier===t.modifier&&e.keycode===t.keycode:e.type==="mouse_click"?e.button===t.button:!1}function Oe(e){const t=A();if(t===null)return;const a=oe(t),n=a.actions[a.actions.length-1];if(n&&n.type!=="delay"&&td(n,e)){n.count=(n.count||1)+(e.count||1),E(t),T();return}a.actions.length&&a.actions[a.actions.length-1].type!=="delay"&&a.actions.push({type:"delay",ms:po}),a.actions.push({...e,count:e.count||1}),E(t),T()}function Oa(){const e=A(),t=e===null?null:I(e);return t?(t.actions.length||t.actions.push({type:"open_app",target:""}),t.actions[0]):null}function ad(e){const t=Oa();t&&(t.kind=e,E(A()),T())}function wo(){const e=A(),t=e===null?null:I(e);return t?(t.actions.length||t.actions.push({type:"text",text:""}),t.actions[0]):null}function nd(e){const t=J(),a=t.modifier===M&&Qt(t.keycode)?t.keycode:me(),n=oe(a);n.actions=[{type:"system_power",mode:e}],E(a),K(M,a)}function Mo(e,t,a){const n=J(),i=je(n.modifier===M?n.keycode:-1)?.plugin===e?n.keycode:me(),o=oe(i),c={type:"plugin",plugin:e,op:t};Number.isFinite(a)&&(c.value=a),o.actions=[c],E(i),K(M,i)}function rd(e,t){const a=bt.has(e)?-1:1;return Math.sign(Number(t?.value)||0)===-a}function id(e,t){const a=ar(t?.keycode);if(!a)return!1;const n=bt.has(e)?a.down:a.up;return t.keycode!==n}function od(){const e=O(),t=xe(),a=t===null?void 0:Zt[t];if(!e||t===null||a===void 0)return;const n=U(),r=Pe(e,n,t,d.layer),i=Pe(e,n,a,d.layer);sr(t,{...i},a,{...r})}function mn(e,t,a){const{action:n}=Sa(e,t);if(!n)return;const r=xe();if(n.targets.includes("turn")&&n.step>0&&r!==null&&Zt[r]!==void 0){sd(r,e,t,n);return}const o=_e(),u=(o.type===C.KEY&&o.modifier===M?je(o.keycode):null)?.plugin===e?o.keycode:me(),p=oe(u),h={type:"plugin",plugin:e,op:t};Number.isFinite(a)&&(h.value=a),p.actions=[h],E(u),Xe({type:C.KEY,modifier:M,keycode:u})}function sd(e,t,a,n){const r=Zt[e],i=O(),o=U(),c=bt.has(e)?r:e,u=bt.has(e)?e:r,p=y=>{const m=Pe(i,o,y,d.layer);return(m.type===C.KEY&&m.modifier===M?je(m.keycode):null)?.plugin===t?m.keycode:me()},h=p(c);oe(h).actions=[{type:"plugin",plugin:t,op:a,value:n.step}];const v=p(u);oe(v).actions=[{type:"plugin",plugin:t,op:a,value:-n.step}],E(h),E(v),sr(c,{type:C.KEY,modifier:M,keycode:h},u,{type:C.KEY,modifier:M,keycode:v})}function ld(e){const t=Ta.find(o=>o.id===e),a=xe(),n=a===null?void 0:Zt[a];if(!t||a===null||n===void 0)return;const r=bt.has(a)?n:a,i=bt.has(a)?a:n;sr(r,{type:C.CONSUMER,modifier:0,keycode:t.up},i,{type:C.CONSUMER,modifier:0,keycode:t.down})}async function cd(e){const t=Oa();if(!t)return;let a;try{a=await window.orby.pickAppOrFile(e)}catch{g("El selector de archivos no está disponible","error");return}a?.ok&&(t.target=a.path,t.kind=e,E(A()),T())}async function dd(){const e=Oa();if(!e)return;let t;try{t=await Na()}catch{g("El detector de aplicaciones no está disponible","error");return}if(!t?.path){g("No se ha podido saber qué ejecutable es esa ventana","error");return}e.target=t.path,e.kind="app",E(A()),T(),g(`Añadida "${t.process||t.path}"`)}let Rt=[],oa=!1;function ko(){oa||Rt.length||(oa=!0,window.orby.listInstalledApps().then(e=>{Rt=e||[]}).catch(()=>{Rt=[]}).finally(()=>{oa=!1,Ne()&&(d.tab==="app"||d.tab==="sequence")&&T()}))}function Eo(){return`<datalist id="installed-apps-list">
    ${Rt.map(e=>`<option value="${x(e.target)}" label="${x(e.name)}"></option>`).join("")}
  </datalist>`}function ud(e){const t=e===null?{target:"",kind:"app"}:I(e)?.actions?.[0]||{target:"",kind:"app"},a=t.kind==="file"?"file":"app";return a==="app"&&ko(),`
    <div class="field">
      <span class="field-label">Qué abrir</span>
      <div class="row-inline" style="gap:6px">
        <button class="type-chip ${a==="app"?"on":""}" data-act="app-kind" data-kind="app">Aplicación</button>
        <button class="type-chip ${a==="file"?"on":""}" data-act="app-kind" data-kind="file">Archivo</button>
      </div>
    </div>

    ${a==="app"?`
      <div class="row-inline mt-4" style="gap:6px">
        <button class="secondary-btn" data-act="app-focus">${f("fit",16)} App en foco</button>
        <button class="secondary-btn" data-act="app-browse" data-kind="app">${f("upload",16)} Examinar…</button>
      </div>

      <label class="field mt-4">
        <span class="field-label">Aplicación</span>
        <input type="text" class="text-input" list="installed-apps-list" data-act="app-target"
               value="${x(t.target||"")}"
               placeholder="Escribe para buscar entre las instaladas, o usa los botones de arriba">
        ${Eo()}
        ${oa&&!Rt.length?'<span class="setting-desc">Buscando aplicaciones instaladas…</span>':""}
      </label>
    `:`
      <label class="field mt-4">
        <span class="field-label">Archivo</span>
        <div class="row-inline" style="gap:6px">
          <input type="text" class="text-input" style="flex:1" data-act="app-target"
                 value="${x(t.target||"")}" placeholder="Ruta del archivo">
          <button class="secondary-btn" data-act="app-browse" data-kind="file">
            ${f("upload",16)} Examinar…
          </button>
        </div>
      </label>
    `}

    <p class="setting-desc">
      Se ejecuta en el PC al pulsar la tecla (necesita esta app abierta), igual que un paso
      "Abrir" de una secuencia.
    </p>

    <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar</button>`}function pd(e){const t=e===null?{text:""}:I(e)?.actions?.[0]||{text:""};return`
    <label class="field">
      <span class="field-label">Texto que escribe la tecla</span>
      <textarea class="text-input" rows="4" data-act="text-value"
                placeholder="Por ejemplo tu correo, una firma o un trozo de código">${x(t.text||"")}</textarea>
    </label>

    <p class="setting-desc">
      Lo escribe el PC, así que necesita esta app abierta (vale con el icono de la bandeja).
      Al ir por unicode y no por códigos de tecla, sale igual con cualquier distribución: eñes,
      acentos y símbolos incluidos. Los textos de más de cuatro letras entran de golpe por el
      portapapeles y un Ctrl+V (se restaura lo que tuvieras copiado), así que da igual lo largos
      que sean; donde Ctrl+V no pegue —la consola clásica, algún juego— no aparecerá nada. Los
      saltos de línea se mandan como Intro y los tabuladores como Tab.
    </p>

    <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar</button>`}async function fd(e=null,t="app"){let a;try{a=await window.orby.pickAppOrFile(t)}catch{g("El selector de archivos no está disponible","error");return}if(a?.ok)if(e!==null){const n=Q(e);n&&(n.target=a.path,n.kind=t,E(A()),T())}else Oe({type:"open_app",target:a.path,kind:t})}function hd(e,t){const a=Q(e);a&&(a.kind=t,E(A()),T())}async function md(e){const t=Q(e);if(!t)return;let a;try{a=await Na()}catch{g("El detector de aplicaciones no está disponible","error");return}if(!a?.path){g("No se ha podido saber qué ejecutable es esa ventana","error");return}t.target=a.path,t.kind="app",E(A()),T(),g(`Añadida "${a.process||a.path}"`)}function gd(e){const t=A();if(t===null)return;const a=I(t);a&&(a.actions.splice(e,1),yd(a),E(t),T())}function Vr(e,t){const a=A();if(a===null)return;const n=I(a);if(!n)return;const r=n.actions;if(!r[e]||r[e].type==="delay")return;let i=e+t;for(;r[i]&&r[i].type==="delay";)i+=t;r[i]&&([r[e],r[i]]=[r[i],r[e]],E(a),T())}function yd(e){const t=[];for(const a of e.actions)a.type==="delay"&&(!t.length||t[t.length-1].type==="delay")||t.push(a);t.length&&t[t.length-1].type==="delay"&&t.pop(),e.actions=t}function Q(e){const t=A();return(t===null?null:I(t))?.actions[e]||null}let be={modifier:0,keycode:0},We={x:0,y:0},Ce=null,va=null;function bd(){Ce&&(clearInterval(Ce),Ce=null)}function Fr(e=null){d.capturing="position",va=e,T(),Ce&&clearInterval(Ce),Ce=setInterval(async()=>{try{const t=await window.orby.getMousePosition();We=t;const a=document.getElementById("seq-live-pos");a&&(a.textContent=`(${t.x}, ${t.y}) — mueve el ratón y pulsa Esc para fijarla`)}catch{}},80)}function gn(e){Ce&&(clearInterval(Ce),Ce=null),d.capturing=!1;const t=va;if(va=null,!e){T();return}if(t!==null){const a=Q(t);a&&(a.type="mouse_position",a.x=We.x,a.y=We.y,E()),T()}else Oe({type:"mouse_position",x:We.x,y:We.y})}function vd(){if(_e().modifier===M)return;const e=me();oe(e),Xe({type:C.KEY,modifier:M,keycode:e})}function Po(e){const t=e===null?null:I(e),a=t?.actions||[],n=d.capturing==="sequence",r=d.capturing==="position";a.some(y=>y.type==="open_app"&&y.kind!=="file")&&ko();const i={left:"Izquierdo",middle:"Central",right:"Derecho"},o=a.reduce((y,m,k)=>m.type==="delay"?y:k,-1),c=(y,m,k)=>`
    <button class="tool-btn small" data-act="seq-move-up" data-index="${y}" title="Subir paso" ${m?"disabled":""}>
      ${f("up",14)}
    </button>
    <button class="tool-btn small" data-act="seq-move-down" data-index="${y}" title="Bajar paso" ${k?"disabled":""}>
      ${f("down",14)}
    </button>`,u=(y,m)=>`
    <input type="number" class="text-input compact seq-count-input" min="1" max="99"
           data-act="seq-count" data-index="${m}" value="${y.count||1}" title="Veces">`,p=(y,m)=>y.count>1?`
    <input type="number" class="text-input compact seq-gap-input" min="0" max="32767" step="5"
           data-act="seq-gap-ms" data-index="${m}" value="${y.gap??no}" title="Espera entre repeticiones (ms)">`:"";let h=0;const v=a.map((y,m)=>{if(y.type==="delay")return`
        <li class="seq-item seq-gap">
          <span>${f("reset",13)} Espera</span>
          <span class="row-inline" style="gap:4px">
            <input type="number" class="text-input compact seq-gap-input" min="0" step="10"
                   value="${y.ms}" data-act="seq-delay-ms" data-index="${m}">
            <span class="setting-desc">ms</span>
          </span>
        </li>`;if(h++,y.type==="mouse_position")return`
        <li class="seq-item">
          <span>${h}. Posición de ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-x" data-index="${m}" value="${y.x}" title="x">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-y" data-index="${m}" value="${y.y}" title="y">
            <button class="tool-btn small" data-act="seq-recapture" data-index="${m}" title="Recapturar con el ratón">
              ${f("fit",14)}
            </button>
            ${c(m,h===1,m===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="center_mouse")return`
        <li class="seq-item">
          <span>${h}. Centrar ratón</span>
          <span class="row-inline" style="gap:4px">
            ${c(m,h===1,m===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="mouse_move")return`
        <li class="seq-item">
          <span>${h}. Mover ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dx" data-index="${m}" value="${y.dx}" title="dx">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dy" data-index="${m}" value="${y.dy}" title="dy">
            ${c(m,h===1,m===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="mouse_click"){const k=y.button||"left";return`
        <li class="seq-item">
          <span>${h}. Clic ${y.count>1?`×${y.count}`:""}</span>
          <span class="row-inline" style="gap:6px">
            <select class="select-input compact" data-act="seq-click-button" data-index="${m}">
              ${Object.entries(i).map(([N,H])=>`<option value="${N}" ${k===N?"selected":""}>${H}</option>`).join("")}
            </select>
            ${u(y,m)}
            ${p(y,m)}
            ${c(m,h===1,m===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`}if(y.type==="text")return`
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${h}. Escribir texto ${y.count>1?`×${y.count}`:""}</span>
            <span class="row-inline" style="gap:4px">
              ${u(y,m)}
              ${p(y,m)}
              ${c(m,h===1,m===o)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
                ${f("trash",14)}
              </button>
            </span>
          </div>
          <textarea class="text-input mt-4" rows="2" style="width:100%"
                    data-act="seq-text" data-index="${m}"
                    placeholder="Texto que se escribirá">${x(y.text||"")}</textarea>
        </li>`;if(y.type==="open_app"){const k=y.kind==="file"?"file":"app";return`
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${h}. Abrir</span>
            <span class="row-inline" style="gap:4px">
              ${c(m,h===1,m===o)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
                ${f("trash",14)}
              </button>
            </span>
          </div>
          <div class="row-inline mt-4" style="gap:6px">
            <button class="type-chip small ${k==="app"?"on":""}" data-act="seq-open-kind" data-index="${m}" data-kind="app">Aplicación</button>
            <button class="type-chip small ${k==="file"?"on":""}" data-act="seq-open-kind" data-index="${m}" data-kind="file">Archivo</button>
          </div>
          ${k==="app"?`
            <div class="row-inline mt-4" style="gap:6px">
              <button class="tool-btn small" data-act="seq-open-focus" data-index="${m}" title="Usar la app en primer plano ahora mismo">
                ${f("fit",14)} App en foco
              </button>
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${m}" data-kind="app" title="Examinar…">
                ${f("upload",14)}
              </button>
            </div>
            <input type="text" class="text-input compact mt-4" style="width:100%" list="installed-apps-list"
                   data-act="seq-open-target" data-index="${m}" value="${x(y.target||"")}"
                   placeholder="Escribe para buscar entre las instaladas" title="${x(y.target||"")}">
          `:`
            <div class="row-inline mt-4" style="gap:6px">
              <input type="text" class="text-input compact" style="flex:1;min-width:120px"
                     data-act="seq-open-target" data-index="${m}" value="${x(y.target||"")}"
                     placeholder="Ruta del archivo" title="${x(y.target||"")}">
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${m}" data-kind="file" title="Examinar…">
                ${f("upload",14)}
              </button>
            </div>
          `}
        </li>`}return y.type==="hotkey"?`
        <li class="seq-item">
          <span>${h}. ${x(_t(y.modifier,y.keycode))} ${y.count>1?`×${y.count}`:""}</span>
          <span class="row-inline" style="gap:4px">
            ${u(y,m)}
            ${p(y,m)}
            ${c(m,h===1,m===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`:`
      <li class="seq-item">
        <span>${h}. Tecla ${x(y.code)}</span>
        <span class="row-inline" style="gap:4px">
          ${c(m,h===1,m===o)}
          <button class="tool-btn danger small" data-act="seq-del" data-index="${m}" title="Quitar este paso">
            ${f("trash",14)}
          </button>
        </span>
      </li>`}).join("");return`
    <div class="field">
      <span class="field-label">Pasos de la secuencia</span>
      ${a.length?`<ul class="seq-list">${v}</ul>`:'<p class="setting-desc">Todavía no tiene ningún paso.</p>'}
      ${Eo()}

      <div class="row-inline mt-4">
        <button class="secondary-btn ${r?"is-capturing":""} ${V("pcSequences")?"":"unsupported"}"
                ${V("pcSequences")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-position">
          ${f("fit",16)} ${r?va!==null?"Recapturando… (Esc fija)":"Capturando… (Esc fija)":"Posición de ratón"}
        </button>
        <button class="secondary-btn" data-act="seq-add-click">${f("bolt",16)} Clic</button>
        <button class="secondary-btn" data-act="seq-add-move">${f("reset",16)} Mover ratón</button>
        <button class="secondary-btn ${V("text")?"":"unsupported"}"
                ${V("text")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-text">${f("pencil",16)} Escribir texto</button>
        <button class="secondary-btn ${V("openApp")?"":"unsupported"}"
                ${V("openApp")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-open">${f("upload",16)} Abrir app/archivo</button>
      </div>
      ${r?`<p class="setting-desc" id="seq-live-pos">(${We.x}, ${We.y}) — mueve el ratón y pulsa Esc para fijarla</p>`:""}

      <span class="field-label mt-4">Tecla (con modificadores)</span>
      <div class="mod-grid">
        ${Aa.map(y=>`
          <button class="mod-chip ${be.modifier&y.bit?"on":""}"
                  data-act="seq-key-mod" data-bit="${y.bit}">${y.label}</button>`).join("")}
      </div>
      <div class="row-inline mt-4">
        <select class="select-input" data-act="seq-key-pick" style="flex:1">
          <option value="0" ${be.keycode?"":"selected"}>— ninguna —</option>
          ${dr(be.keycode)}
        </select>
        <button class="secondary-btn" data-act="seq-add-hotkey">${f("plus",16)} Añadir</button>
      </div>

      <button class="primary-btn full mt-4 ${n?"is-capturing":""}" data-act="seq-record">
        ${f("key",16)} ${n?"Grabando… pulsa teclas (Esc termina)":"Grabar secuencia de teclas"}
      </button>

      ${t?$d(t):""}

      <p class="setting-desc">
        "Grabar secuencia" solo reconoce letras, dígitos, Enter y Espacio; para el resto de
        teclas o combinaciones con modificadores usa "Tecla" arriba, y para meter un texto tal
        cual (correo, firma, un trozo de código) "Escribir texto". Entre cada dos pasos se
        espera automáticamente lo que pongas en "Espera" (${po} ms por
        defecto suele bastar).
      </p>

      <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar secuencia</button>
    </div>`}function $d(e){if(zt(e))return`<p class="setting-desc">${f("check",13)} Se ejecuta en el propio teclado: sigue
              funcionando aunque cierres esta app.</p>`;const t=(e.actions||[]).some(i=>i.type==="open_app"),a=(e.actions||[]).some(i=>i.type==="text"),n=(e.actions||[]).some(i=>i.type==="mouse_position"||i.type==="center_mouse");return`<p class="setting-desc">Se ejecuta en el PC, no en el teclado (${t?"abre una app o un archivo, algo que solo sabe hacer el PC":a?"escribe un texto, y eso el PC lo manda como unicode: el teclado solo sabe mandar códigos de tecla, que cambian con la distribución":n?"usa una posición de ratón absoluta, que de momento solo sabe reproducir el PC":`tiene más de ${to} pasos, o alguno de un formato antiguo`}): solo
            funciona con esta app abierta.</p>`}function Co(){Nc({render:P,renderKeyGrid:Yr});const e=document.getElementById("profiles-body");e.addEventListener("click",xd),e.addEventListener("change",wd),e.addEventListener("input",Md),window.addEventListener("keydown",kd,!0),e.addEventListener("keydown",r=>{r.target.id==="variant-new-match"&&r.key==="Enter"&&(r.preventDefault(),So())}),Fn(()=>{if(!Ne())return;Yr();const r=document.getElementById("profile-bar");if(!r)return;const i=r.querySelectorAll(".tab-icon-canvas").length,o=s.profiles.filter((c,u)=>Vn(u)).length;i!==o?Ao():Xt(r)}),yt().then(()=>{Ne()&&d.selected&&P()}),Jn(()=>{Ne()&&P()}),Ri(r=>{r==="applied"&&Ne()&&P()}),window.orby.recorder.onState(r=>{Fc(r).catch(i=>console.error("Grabación:",i))});let t=-1,a=-1,n=-1;it(()=>{s.profiles.length===t&&s.activeProfileIdx===a&&s.pageIdx===n||(t=s.profiles.length,a=s.activeProfileIdx,n=s.pageIdx,Ne()&&P())})}function xd(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;if(a==="pick-profile")d.editingProfile=Number(t.dataset.idx),d.variantId=null,d.selected=null,P();else if(a==="pick-variant")d.variantId=t.dataset.id||null,d.selected=null,P();else if(a==="new-variant")Ed();else if(a==="del-variant")Cd();else if(a==="add-match")So();else if(a==="add-match-current")Pd();else if(a==="del-match")vl(d.variantId,t.dataset.match),P();else if(a==="clear-override")Sd();else if(a==="layer")d.layer=t.dataset.layer,d.selected=null,P();else if(a==="page")Rd(Number(t.dataset.page));else if(a==="page-add")Nd();else if(a==="page-del")Od(Number(t.dataset.page));else if(a==="pick-key")d.selected={kind:"key",index:Number(t.dataset.key)},d.capturing=!1,d.tab=Io(J()),P();else if(a==="pick-rotary")d.selected={kind:"rotary",slot:Number(t.dataset.slot)},d.capturing=!1,P();else if(a==="edit-icon")rn("view-oled",{profile:d.editingProfile,key:Number(t.dataset.key),layer:d.layer});else if(a==="copy-key")qc();else if(a==="paste-key")_c();else if(a==="rotary-type")Xe({type:Number(t.dataset.type),modifier:0,keycode:0});else if(a==="rotary-consumer")Xe({type:C.CONSUMER,modifier:0,keycode:Number(t.dataset.index)});else if(a==="rotary-consumer-pair")ld(t.dataset.pair);else if(a==="rotary-mod"){const n=_e();Xe({type:C.KEY,modifier:n.modifier^Number(t.dataset.bit),keycode:n.keycode})}else if(a==="activate")ci(d.editingProfile).then(()=>{s.activeProfileIdx=d.editingProfile,te(),P()}).catch(()=>g("El teclado no confirmó el cambio de perfil","error"));else if(a==="capture")d.capturing=!d.capturing,P();else if(a==="clear-action")Ld();else if(a==="set-goto-page")K(et,Number(t.dataset.page));else if(a==="set-page-state")K(tt,0);else if(a==="set-consumer")K(gt,Number(t.dataset.index));else if(a==="set-power")nd(t.dataset.mode);else if(a==="toggle-mod"){const n=Number(t.dataset.bit),r=J();r.modifier>=gt||r.modifier===et||r.modifier===tt?K(n,0):K(r.modifier^n,r.keycode)}else if(a==="profile-new")Xr(null);else if(a==="profile-dup")Xr(d.editingProfile);else if(a==="profile-del")Id();else if(a==="profile-icon")rn("view-oled",{profile:d.editingProfile,kind:"profile"});else if(a==="scroll-preset")bn({detentsPerRev:Number(t.dataset.value)});else if(a==="scroll-invert")bn({invert:!or().invert});else if(a==="set-tab")Ad(t.dataset.tab);else if(a==="seq-add-position")d.capturing==="position"?gn(!1):Fr();else if(a==="seq-recapture")d.capturing==="position"?gn(!1):Fr(Number(t.dataset.index));else if(a==="seq-add-click")Oe({type:"mouse_click",button:"left"});else if(a==="seq-add-open")Oe({type:"open_app",target:"",kind:"app"});else if(a==="seq-add-text")Oe({type:"text",text:""});else if(a==="seq-open-browse")fd(Number(t.dataset.index),t.dataset.kind);else if(a==="seq-open-kind")hd(Number(t.dataset.index),t.dataset.kind);else if(a==="seq-open-focus")md(Number(t.dataset.index));else if(a==="app-kind")ad(t.dataset.kind);else if(a==="app-browse")cd(t.dataset.kind);else if(a==="app-focus")dd();else if(a==="seq-add-move")Oe({type:"mouse_move",dx:10,dy:0});else if(a==="seq-key-mod")be.modifier^=Number(t.dataset.bit),P();else if(a==="seq-add-hotkey"){if(!be.modifier&&!be.keycode)return;Oe({type:"hotkey",modifier:be.modifier,keycode:be.keycode}),be.keycode=0}else if(a==="seq-del")gd(Number(t.dataset.index));else if(a==="seq-move-up")Vr(Number(t.dataset.index),-1);else if(a==="seq-move-down")Vr(Number(t.dataset.index),1);else if(a==="seq-record")d.capturing=d.capturing==="sequence"?!1:"sequence",P();else if(a==="seq-clear")d.tab="shortcut",K(0,0);else if(a==="rotary-macro")vd();else if(a==="set-plugin")Mo(t.dataset.plugin,t.dataset.op);else if(a==="rotary-plugin")mn(t.dataset.plugin,t.dataset.op);else if(a==="plugin-value-open"){const n=t.dataset.mode,r=_e(),i=n==="rotary"?r.type===C.KEY&&r.modifier===M?r.keycode:null:A(),o=i!=null?je(i):null,c=o?.plugin===t.dataset.plugin&&o.op===t.dataset.op?o.value:void 0;Qc(t.dataset.plugin,t.dataset.op,n,c)}else if(a==="plugin-value-confirm")ed();else if(a==="plugin-value-cancel")Jc();else if(a==="rotary-invert")od();else if(a==="rotary-plugin-tab"){const n=t.dataset.plugin,r=_e();if((r.modifier===M?je(r.keycode):null)?.plugin!==n){const o=!!fo(xe())?.part.discrete,c=Ca(Je(n),o?"click":"turn")[0];c&&mn(n,c.op)}}else a==="rec-toggle"?Kc():a==="rec-mode"?Wc(t.dataset.mode):a==="rec-speed"?Uc(t.dataset.speed):a==="rec-clear"?Gc():a==="rec-stop"&&window.orby.recorder.stop()}function wd(e){const t=e.target.dataset.act;if(t==="pick-keycode"){const a=J(),r=a.modifier===gt||a.modifier===et||a.modifier===tt||a.modifier===M?0:a.modifier;K(r,Number(e.target.value))}else if(t==="rotary-keycode"){const a=_e();Xe({type:C.KEY,modifier:a.modifier,keycode:Number(e.target.value)})}else if(t==="scroll-slider")bn({detentsPerRev:Number(e.target.value)});else if(t==="variant-field")kr(d.variantId,{field:e.target.value});else if(t==="variant-name")kr(d.variantId,{name:e.target.value.trim()}),P();else if(t==="seq-click-button"){const a=Q(Number(e.target.dataset.index));a&&(a.button=e.target.value,E(A()))}else if(t==="seq-count"){const a=Q(Number(e.target.dataset.index));a&&(a.count=Math.max(1,Math.min(99,Math.round(Number(e.target.value))||1)),E(A()),P())}else if(t==="seq-delay-ms"){const a=Q(Number(e.target.dataset.index));a&&(a.ms=Math.max(0,Number(e.target.value)||0),E(A()))}else if(t==="seq-gap-ms"){const a=Q(Number(e.target.dataset.index));a&&(a.gap=Math.max(0,Math.min(32767,Math.round(Number(e.target.value))||0)),E(A()))}else if(t==="seq-pos-x"){const a=Q(Number(e.target.dataset.index));a&&(a.x=Math.round(Number(e.target.value)||0),E(A()))}else if(t==="seq-pos-y"){const a=Q(Number(e.target.dataset.index));a&&(a.y=Math.round(Number(e.target.value)||0),E(A()))}else if(t==="seq-open-target"){const a=Q(Number(e.target.dataset.index));a&&(a.target=e.target.value,E(A()))}else if(t==="seq-text"){const a=Q(Number(e.target.dataset.index));a&&(a.text=e.target.value,Gr(A()))}else if(t==="text-value"){const a=wo();a&&(a.text=e.target.value,Gr(A()))}else if(t==="app-target"){const a=Oa();a&&(a.target=e.target.value,E(A()))}else if(t==="seq-move-dx"){const a=Q(Number(e.target.dataset.index));a&&(a.dx=Wr(e.target.value),E(A()))}else if(t==="seq-move-dy"){const a=Q(Number(e.target.dataset.index));a&&(a.dy=Wr(e.target.value),E(A()))}else t==="seq-key-pick"&&(be.keycode=Number(e.target.value))}function Wr(e){const t=Math.round(Number(e)||0);return Math.max(-127,Math.min(127,t))}let yn=null;function Ur(e){clearTimeout(yn),yn=setTimeout(()=>E(e),400)}function Gr(e){clearTimeout(yn),E(e),P()}let Kr=null;function Md(e){const t=e.target.dataset.act;if(t==="seq-text"){const a=Q(Number(e.target.dataset.index));a&&(a.text=e.target.value,Ur(A()))}else if(t==="text-value"){const a=wo();a&&(a.text=e.target.value,Ur(A()))}else if(t==="edit-name"){const a=e.target.value.slice(0,7);clearTimeout(Kr),Kr=setTimeout(async()=>{const n=O();if(!(!n||!Y())){n.name=a;try{await ui(d.editingProfile,a),B(),te(),Ao()}catch{g("El teclado no confirmó el nombre","error")}}},250)}else if(t==="plugin-value-slider"){const a=Number(e.target.value);Zc(a);const n=document.getElementById("plugin-value-readout");n&&(n.textContent=a)}else if(t==="scroll-slider"){const a=Number(e.target.value),n=document.getElementById("scroll-value");n&&(n.textContent=a),No(a)}}function kd(e){if(!d.capturing||!d.selected)return;if(d.capturing==="position"){e.key==="Escape"&&(e.preventDefault(),e.stopPropagation(),gn(!0));return}if(e.preventDefault(),e.stopPropagation(),e.key==="Escape"){d.capturing=!1,P();return}if(d.capturing==="sequence"){const a=Ar(e);if(!a.keycode)return;Oe({type:"hotkey",modifier:a.modifier,keycode:a.keycode});return}const t=Ar(e);t.keycode&&(d.capturing=!1,d.selected.kind==="rotary"?Xe({type:C.KEY,modifier:t.modifier,keycode:t.keycode}):K(t.modifier,t.keycode))}async function Ed(){let e="";try{e=((await Na())?.process||"").toLowerCase()}catch{}const t=bl(d.editingProfile,{page:ba(),name:e?e.replace(/\.exe$/,""):"Variación",matches:e?[e]:[]});d.variantId=t.id,d.selected=null,P(),g(e?`Variación creada para "${e}"`:"Variación creada: indícale a qué app se aplica")}function So(){const e=document.getElementById("variant-new-match");if(!e)return;const t=e.value.trim();if(t){if(!Ni(d.variantId,t)){g("Esa aplicación ya está en la lista","info");return}e.value="",P(),document.getElementById("variant-new-match")?.focus()}}async function Pd(){try{const t=((await Na())?.process||"").toLowerCase();if(!t){g("Aún no se ha detectado ninguna ventana","error");return}if(!Ni(d.variantId,t)){g("Esa aplicación ya está en la lista","info");return}P()}catch{g("El detector de aplicaciones no está disponible","error")}}async function Cd(){const e=U();e&&confirm(`Se borrará la variación "${e.name}" y sus ${Ot(e)} cambios.

El perfil base no se toca.

¿Continuar?`)&&(await $l(e.id),d.variantId=null,d.selected=null,P(),g("Variación eliminada"))}async function Sd(){const e=U();if(!e||!d.selected)return;const t=d.selected.kind==="rotary"?"rotary":"keys",a=d.selected.kind==="rotary"?pe(d.selected.slot,d.layer):le(d.selected.index,d.layer);if(Er(e.id,t,a),d.selected.kind==="key"&&Er(e.id,"labels",F(d.selected.index,d.layer)),Te(e.id))try{if(t==="keys"){await za(e.id,"keys",a);const n=F(d.selected.index,d.layer);n>=0&&await za(e.id,"labels",n)}else await za(e.id,"rotary",a)}catch{g("El teclado no confirmó la vuelta al valor base","error")}d.selected.kind==="key"&&(d.tab=Io(J())),P()}async function Xr(e){if(!d.busy){if(!s.connected){g("Teclado no conectado","error");return}if(s.profiles.length>=s.maxProfiles){g(`El teclado admite como máximo ${s.maxProfiles} perfiles`,"error");return}d.busy=!0,P();try{const t=e===null?await pi():await vs(e);ka(),await fe(),d.editingProfile=Number.isInteger(t)?t:s.profiles.length-1,d.selected=null,B(),g(e===null?"Perfil creado":"Perfil duplicado")}catch(t){g(`No se pudo crear el perfil: ${t.message}`,"error")}finally{d.busy=!1,P()}}}async function Id(){if(d.busy)return;if(!s.connected){g("Teclado no conectado","error");return}if(s.profiles.length<=1){g("Tiene que quedar al menos un perfil","error");return}const e=O();if(confirm(`Se borrará el perfil "${e?.name||""}" con sus atajos, mandos e iconos.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`)){d.busy=!0,P();try{await $s(d.editingProfile),wl(d.editingProfile),d.variantId=null,ka(),await fe(),d.editingProfile=Math.min(d.editingProfile,s.profiles.length-1),d.selected=null,B(),g("Perfil eliminado")}catch(t){g(`No se pudo eliminar: ${t.message}`,"error")}finally{d.busy=!1,P()}}}async function Ld(){const e=J();if(e.modifier===M&&At(e.keycode)){const t=ct(e.keycode);t&&await $o(t.id)}await K(0,0),await Oc()}function Io(e){return e.modifier===gt?"media":e.modifier===et||e.modifier===tt?"pages":e.modifier===M?At(e.keycode)?"record":Qt(e.keycode)||wt(e.keycode)?"media":ma(e.keycode)?"text":ha(e.keycode)?"app":"sequence":"shortcut"}function Ad(e){const t=d.tab;if(d.tab=e,t==="record"&&e!=="record"){const a=J();if(a.modifier===M&&At(a.keycode)){const n=ct(a.keycode);n&&$o(n.id)}}if(e==="sequence"&&t!=="sequence"){const a=J();if(a.modifier!==M||ha(a.keycode)||Qt(a.keycode)||wt(a.keycode)||ma(a.keycode)||At(a.keycode)){const n=me();oe(n),K(M,n);return}}if(e==="text"&&t!=="text"){const a=J();if(a.modifier!==M||!ma(a.keycode)){const n=me(),r=oe(n);r.actions=[{type:"text",text:""}],E(n),K(M,n);return}}if(e==="app"&&t!=="app"){const a=J();if(a.modifier!==M||!ha(a.keycode)){const n=me(),r=oe(n);r.actions=[{type:"open_app",target:""}],E(n),K(M,n);return}}if(e==="record"&&t!=="record"){const a=J();if(a.modifier!==M||!At(a.keycode)){Yc();return}}P()}async function bn(e){const t=O();if(!t||!Y())return;const a=$i(d.layer),n={...Nn(t,d.layer),...e};t.scroll[a]=n,d.editingProfile===s.activeProfileIdx&&a===(s.superActive?1:0)&&(s.scroll={...s.scroll,...n}),P();try{await di(d.editingProfile,a,n.detentsPerRev,n.invert),B()}catch{g("El teclado no confirmó la calibración de la rueda","error")}}function P(){const e=document.getElementById("profiles-body");if(!e)return;const t=e.querySelector(".editor-main")?.scrollTop,a=e.querySelector(".editor-inspector")?.scrollTop;if(d.capturing!=="position"&&bd(),!s.profiles.length){e.innerHTML=`<div class="empty-panel glass-panel">
      ${f("plug",40)}
      <h3>Sin perfiles cargados</h3>
      <p>Conecta el Orby una vez: a partir de ahí queda una copia en el PC y podrás
         editarlos aunque no lo tengas enchufado.</p>
    </div>`;return}d.editingProfile>=s.profiles.length&&(d.editingProfile=0);const n=O();if(e.innerHTML=`
    <div class="profile-bar" id="profile-bar">${Lo()}</div>
    ${Td()}

    <div class="editor-layout">
      <div class="editor-main">
        ${zd()}
        <div class="editor-board glass-panel">
          <div class="editor-board-head">
            <div class="head-row">
              <label class="field-inline">
                <span>Nombre</span>
                <input type="text" maxlength="7" value="${x(n.name)}"
                       data-act="edit-name" class="text-input compact">
              </label>
              ${d.editingProfile===s.activeProfileIdx?'<span class="pill pill-live">Perfil activo</span>':'<button class="secondary-btn" data-act="activate">Activar en el teclado</button>'}
            </div>
            <div class="head-row">
              <div class="layer-toggle">
                <button class="toggle-btn ${d.layer==="normal"?"active":""}" data-act="layer" data-layer="normal">NORMAL</button>
                <button class="toggle-btn ${d.layer==="super"?"active":""}" data-act="layer" data-layer="super">SUPER</button>
              </div>
              ${qd()}
            </div>
          </div>
          ${_d()}

          <div class="okey-grid" id="profile-key-grid">${To()}</div>
          <p class="grid-status" id="profile-grid-status"></p>
        </div>

        <div class="glass-panel oled-card">
          <div class="card-header">${f("reset",20)}<h2>Encoders</h2></div>
          <div class="rotary-groups">${jd()}</div>
          <p class="setting-desc mt-4">
            Cada capa guarda sus propias acciones: con <strong>SUPER</strong> mantenida los encoders
            hacen lo que configures aquí en la capa SUPER. Dentro del menú del teclado siguen
            sirviendo para navegar.
          </p>
        </div>

        ${Dd()}
      </div>

      ${Hd()}
    </div>`,Xt(e),Ro(),No(or().detentsPerRev),De(d.editingProfile),t!==void 0){const r=e.querySelector(".editor-main");r&&(r.scrollTop=t)}if(a!==void 0){const r=e.querySelector(".editor-inspector");r&&(r.scrollTop=a)}}function Lo(){const e=s.profiles.length>=s.maxProfiles,t=lt(s.deviceInfo,"profileIcon");return`
    <div class="profile-tabs">
      ${s.profiles.map((a,n)=>`
        <button class="profile-tab ${n===d.editingProfile?"active":""} ${n===s.activeProfileIdx?"is-live":""}"
                data-act="pick-profile" data-idx="${n}">
          <span class="tab-icon" style="--accent:${la(n).accent}">${Vn(n)?`<canvas class="tab-icon-canvas" data-bmp="${n}:0:20"></canvas>`:f(la(n).icon,18)}</span>
          <span class="tab-name">${x(a.name)}</span>
          ${n===s.activeProfileIdx?'<span class="tab-live">EN USO</span>':""}
        </button>`).join("")}
    </div>
    <div class="profile-bar-actions">
      <button class="secondary-btn" data-act="profile-new" ${e||d.busy?"disabled":""}
              title="${e?`Máximo ${s.maxProfiles} perfiles`:"Crear un perfil vacío"}">
        ${f("plus",16)} Nuevo
      </button>
      <button class="secondary-btn" data-act="profile-dup" ${e||d.busy?"disabled":""}
              title="Copiar el perfil actual con sus iconos">
        ${f("profiles",16)} Duplicar
      </button>
      <button class="secondary-btn danger" data-act="profile-del"
              ${s.profiles.length<=1||d.busy?"disabled":""}>
        ${f("trash",16)} Eliminar
      </button>
      <button class="secondary-btn" data-act="profile-icon" ${t?"":"disabled"}
              title="${t?"Icono que enseña este perfil en el menú del teclado":`Necesita firmware ${tr.profileIcon.since} o posterior`}">
        ${f("oled",16)} Icono del perfil
      </button>
      <span class="profile-count">${s.profiles.length} / ${s.maxProfiles}</span>
    </div>`}function Ao(){const e=document.getElementById("profile-bar");e&&(e.innerHTML=Lo(),Xt(e))}function Td(){const e=gl(d.editingProfile,ba()),t=s.profiles[d.editingProfile],a=rt()&&t&&D(t)>1?` · página ${ba()+1}`:"";return`
    <div class="variant-bar">
      <span class="variant-bar-label">${f("bolt",14)} Según la app${a}</span>
      <div class="chip-row">
        <button class="chip ${d.variantId?"":"on"}" data-act="pick-variant" data-id="">
          Perfil base
        </button>
        ${e.map(n=>`
          <button class="chip ${d.variantId===n.id?"on":""} ${Te(n.id)?"is-live":""}"
                  data-act="pick-variant" data-id="${n.id}"
                  title="${x(n.matches.length?`Se aplica con: ${n.matches.join(", ")}`:"Sin aplicaciones asignadas")}">
            ${x(n.name)}
            <em class="chip-count">${Ot(n)}</em>
          </button>`).join("")}
        <button class="chip ghost" data-act="new-variant">${f("plus",14)} Nueva variación</button>
      </div>
    </div>`}async function Rd(e){const t=s.profiles[d.editingProfile];if(!(!t||e===(t.pageIdx||0))){if(s.connected&&d.editingProfile!==s.activeProfileIdx){g("Para editar otra página, activa antes este perfil en el teclado");return}if(d.selected=null,d.variantId=null,!s.connected){t.pageIdx=e,P();return}try{await xi(e),De(d.editingProfile),P()}catch(a){g(`No he podido cambiar de página: ${a.message}`),P()}}}async function Nd(){if(!(!s.profiles[d.editingProfile]||!Y())){if(d.editingProfile!==s.activeProfileIdx){g("Activa este perfil en el teclado para añadirle páginas");return}try{if(await qs()){const t=s.profiles[d.editingProfile];await xi(D(t)-1),De(d.editingProfile),g(`Página ${D(t)} añadida, vacía`)}else g(`El tope es de ${Ut()} páginas por perfil`);P()}catch(t){g(`No he podido añadir la página: ${t.message}`)}}}async function Od(e){const t=s.profiles[d.editingProfile];if(!(!t||D(t)<=1||!Y())){if(d.editingProfile!==s.activeProfileIdx){g("Activa este perfil en el teclado para borrarle páginas");return}if(confirm(`¿Eliminar la página ${e+1} de ${x(t.name)}?

Se van sus teclas, etiquetas, mandos e iconos. Las páginas siguientes se recolocan.`))try{await _s(e),xl(d.editingProfile,e),De(d.editingProfile),d.selected=null,d.variantId=null,g(`Página ${e+1} eliminada`),P()}catch(a){g(`No he podido eliminar la página: ${a.message}`)}}}function qd(){if(!rt())return"";const e=s.profiles[d.editingProfile];if(!e)return"";const t=D(e),a=e.pageIdx||0;let n="";for(let i=0;i<t;i++)n+=`<button class="toggle-btn ${i===a?"active":""}"
                     data-act="page" data-page="${i}" title="Página ${i+1}">${i+1}</button>`;const r=t<Ut();return`
    <div class="layer-toggle page-toggle">
      ${n}
      ${r?`<button class="toggle-btn page-add" data-act="page-add"
                          title="Añadir una página copiando la actual">+</button>`:""}
      ${t>1?`<button class="toggle-btn page-del" data-act="page-del" data-page="${a}"
                             title="Eliminar la página ${a+1}">−</button>`:""}
    </div>`}function _d(){if(!rt())return"";const e=s.profiles[d.editingProfile];if(!e)return"";const t=D(e);if(t<=1)return`<p class="grid-status">Este perfil tiene una sola página. Con
      <strong>+</strong> añades otra en blanco, y el teclado alterna entre ellas
      con una pulsación corta del botón de menú.</p>`;const a=d.editingProfile===s.activeProfileIdx;return`<p class="grid-status">Editando la <strong>página ${(e.pageIdx||0)+1}</strong>
    de ${t}. Cada página tiene sus teclas, etiquetas, mandos, rueda e iconos.
    ${a?"La página que elijas aquí es la que se pone en el teclado, así que las pantallas enseñan lo que estás tocando.":""}</p>`}function zd(){const e=U();if(!e)return"";const t=Te(e.id),a=Ot(e);return`
    <div class="glass-panel oled-card variant-card">
      <div class="card-header">
        ${f("bolt",20)}<h2>Variación de ${x(O().name)}</h2>
        <span class="pill ${t?"pill-live":""}">${t?"Aplicada ahora":`${a} cambios`}</span>
      </div>

      <div class="row-inline">
        <label class="field">
          <span class="field-label">Nombre</span>
          <input type="text" class="text-input" value="${x(e.name)}" data-act="variant-name">
        </label>
        <label class="field">
          <span class="field-label">Comparar contra</span>
          <select class="select-input" data-act="variant-field">
            <option value="any"     ${e.field==="any"?"selected":""}>Programa o título</option>
            <option value="process" ${e.field==="process"?"selected":""}>Solo programa</option>
            <option value="title"   ${e.field==="title"?"selected":""}>Solo título</option>
          </select>
        </label>
      </div>

      <div class="field">
        <span class="field-label">Se aplica con estas aplicaciones</span>
        ${e.matches.length?`
          <div class="chip-row match-list">
            ${e.matches.map(n=>`
              <span class="match-chip">
                ${x(n)}
                <button class="match-x" data-act="del-match" data-match="${x(n)}"
                        title="Quitar">${f("close",12)}</button>
              </span>`).join("")}
          </div>`:`
          <p class="setting-desc">Todavía ninguna: la variación no se aplicará sola.</p>`}

        <div class="row-inline">
          <input type="text" class="text-input compact match-input" id="variant-new-match"
                 placeholder="p. ej. notepad" spellcheck="false">
          <button class="secondary-btn" data-act="add-match">${f("plus",16)} Añadir</button>
          <button class="secondary-btn" data-act="add-match-current">${f("plug",16)} La app de delante</button>
        </div>
      </div>

      <p class="setting-desc">
        Estás editando <strong>solo las diferencias</strong>: lo que cambies aquí se
        guarda como excepción de este perfil para esas apps, y todo lo demás lo sigue
        mandando el perfil base. Las teclas y mandos redefinidos salen marcados abajo.
        ${a===0?"<br>Todavía no hay ninguna diferencia: cambia una tecla para empezar.":""}
      </p>

      <div class="row-inline">
        <button class="secondary-btn danger" data-act="del-variant">${f("trash",16)} Eliminar variación</button>
      </div>
    </div>`}function To(){const e=O(),t=U(),a=Re();let n="";for(let r=0;r<12;r++){const i=ze[r],o=F(r,d.layer),c=zi(e,t,r,d.layer),u=o>=0?Bi(e,t,r,d.layer):"",p=o>=0?ye(d.editingProfile,o):null,h=c.modifier||c.keycode,v=so(c),y=t&&(he(t,"keys",le(r,d.layer))||he(t,"labels",o));n+=`
      <button class="okey ${a===r?"selected":""} ${p?"has-icon":""} ${i?"":"roleless"} ${y?"is-override":""}"
              data-act="pick-key" data-key="${r}">
        <span class="okey-num">T${r+1}${i?`<em>P${i}</em>`:""}${y?'<i class="okey-dot" title="Cambiado en esta variación"></i>':""}</span>
        <span class="okey-screen">
          ${i?p?`<canvas class="okey-canvas" data-bmp="${ue(d.editingProfile,o)}"></canvas>`:`<span class="okey-text">${x(u||"—")}</span>`:`<span class="okey-role">${r+1===ht?"SUPER":"MENÚ"}</span>`}
        </span>
        <span class="okey-action ${h?"assigned":""}">${x(gc(c))}${v?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esta tecla no hace nada">${f("plug",11)}</i>`:""}</span>
      </button>`}return n}function Ro(){const e=document.getElementById("profile-key-grid");e&&Xt(e);const t=document.getElementById("profile-grid-status");t&&(t.textContent=nl()?"Leyendo iconos del teclado…":`Capa ${d.layer==="super"?"SUPER":"NORMAL"} · las teclas ${tn} y ${ht} no tienen pantalla: la ${ht} es el modificador SUPER y la ${tn} abre el menú al mantenerla.`)}function Yr(){const e=document.getElementById("profile-key-grid");e&&(e.innerHTML=To(),Ro())}function Bd(e){return e?.type===C.KEY&&e.modifier===M&&wt(e.keycode)}function vn(e){return!e?.type||nr(e.type)||Bd(e)?!0:e.type===C.CONSUMER&&!!ar(e.keycode)}function jd(){const e=O(),t=U(),a=xe(),n=(r,i,o)=>`
    <button class="rotary-part ${a===r.slot?"selected":""} ${i.type?"assigned":""} ${o?"is-override":""}"
            data-act="pick-rotary" data-slot="${r.slot}">
      <span class="rp-dir">${r.short}</span>
      <span class="rp-body">
        <em>${r.label}</em>
        <strong>${x(Bt(i))}${fn(i)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong>
      </span>
    </button>`;return uo.map(r=>{const[i,o,c]=r.parts,u=Pe(e,t,i.slot,d.layer),p=Pe(e,t,o.slot,d.layer),h=vn(u)&&vn(p),v=t&&he(t,"rotary",pe(i.slot,d.layer)),y=t&&he(t,"rotary",pe(o.slot,d.layer)),m=Pe(e,t,c.slot,d.layer),k=t&&he(t,"rotary",pe(c.slot,d.layer)),N=h?n({slot:i.slot,label:"Giro",short:"⟳"},u,v):n(i,u,v)+n(o,p,y);return`
    <div class="rotary-group">
      <span class="rotary-group-name">${f(r.icon,14)} ${r.name}</span>
      ${N}
      ${n(c,m,k)}
    </div>`}).join("")}function Dd(){const e=or(),t=d.layer==="super"?"SUPER":"normal",a=O(),n=U(),r=Pe(a,n,R.WHEEL_CW,d.layer),i=Pe(a,n,R.WHEEL_CCW,d.layer);return`
    <div class="glass-panel oled-card">
      <div class="card-header">
        ${f("wheel",20)}<h2>Rueda de scroll</h2>
        <span class="pill">Capa ${t}</span>
      </div>

      ${U()?`
        <p class="setting-desc override-note">
          La sensibilidad de la rueda es siempre la del <strong>perfil base</strong>:
          las variaciones cambian atajos y etiquetas, no la calibración.
        </p>`:""}

      <!-- El slider ocupa el ancho completo de la tarjeta, fuera de la rejilla de
           dos columnas: son 234 pasos y cuanto más largo es el recorrido, más
           fino se puede afinar el punto exacto. -->
      <div class="wheel-tune-head">
        <div class="wheel-readout">
          <span id="scroll-value">${e.detentsPerRev}</span>
          <small>clics por vuelta completa</small>
        </div>

        <input type="range" id="scroll-slider" class="premium-slider wheel-slider" data-act="scroll-slider"
               min="3" max="120" step="1" value="${e.detentsPerRev}">

        <div class="slider-row">
          <span>Más lento · 3</span>
          <span>120 · Más rápido</span>
        </div>
      </div>

      <div class="wheel-tune">
        <div class="wheel-tune-main">
          <div class="preset-row">
            ${Tc.map(o=>`
              <button class="preset-btn ${e.detentsPerRev===o.value?"on":""}" data-act="scroll-preset" data-value="${o.value}">
                <strong>${o.name}</strong>
                <span>${o.value} clics</span>
                <em>${o.desc}</em>
              </button>`).join("")}
          </div>

          <ul class="info-list" id="scroll-derived"></ul>

          <div class="row-inline mt-4">
            <button class="secondary-btn ${e.invert?"is-on":""}" data-act="scroll-invert">
              ${f("reset",16)} ${e.invert?"Dirección invertida":"Invertir dirección"}
            </button>
          </div>
        </div>

        <div class="wheel-tune-side">
          <div class="hires-state ${s.scroll.hires?"ok":"off"}">
            <span class="hires-dot"></span>
            <strong>${s.scroll.hires?"Alta resolución activa":"Alta resolución no negociada"}</strong>
          </div>
          <div class="hires-state ${s.scroll.hiresPan?"ok":"off"}">
            <span class="hires-dot"></span>
            <strong>${s.scroll.hiresPan?"Paneo horizontal en alta resolución":"Paneo horizontal sin negociar"}</strong>
          </div>

          <div class="field">
            <span class="field-label">Acción del giro</span>
            <button class="rotary-part ${xe()===R.WHEEL_CW?"selected":""}"
                    data-act="pick-rotary" data-slot="${R.WHEEL_CW}">
              <span class="rp-dir">↓</span>
              <span class="rp-body"><em>Hacia abajo</em><strong>${x(Bt(r))}${fn(r)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong></span>
            </button>
            <button class="rotary-part ${xe()===R.WHEEL_CCW?"selected":""}"
                    data-act="pick-rotary" data-slot="${R.WHEEL_CCW}">
              <span class="rp-dir">↑</span>
              <span class="rp-body"><em>Hacia arriba</em><strong>${x(Bt(i))}${fn(i)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong></span>
            </button>
          </div>

          <p class="setting-desc">
            La sensibilidad pertenece a <strong>${x(O().name)}</strong> en la capa
            ${t}: cada perfil —y cada capa— puede tener la suya. El dibujo de la rueda en
            pantalla (forma del marcador, sentido, desfase) se calibra en <strong>Ajustes</strong>.
            ${s.scroll.hires?"Con el multiplicador negociado el desplazamiento viaja en unidades de 1/120 de clic.":"Hasta que Windows pida el multiplicador, las aplicaciones antiguas saltarán de tres en tres líneas."}
          </p>
        </div>
      </div>
    </div>`}function No(e){const t=document.getElementById("scroll-derived");if(!t)return;const a=360/e,n=e*120/4096;t.innerHTML=`
    <li><span class="lbl">Giro por clic</span><span class="val">${a.toFixed(1)}°</span></li>
    <li><span class="lbl">Unidades HID por cuenta</span><span class="val">${n.toFixed(3)}</span></li>
    <li><span class="lbl">Líneas por vuelta (Windows)</span><span class="val">${(e*3).toFixed(0)}</span></li>`}function Hd(){return d.selected?d.selected.kind==="rotary"?Vd():Fd():`<div class="editor-inspector glass-panel empty-panel">
      ${f("key",36)}
      <h3>Elige una tecla o un mando</h3>
      <p>Selecciona cualquier tecla para cambiar su icono, su etiqueta y su atajo, o un giro
         de los encoders y la rueda para reasignarlo.</p>
    </div>`}function Vd(){const e=xe(),t=fo(e),a=_e(),n=!!t?.part.discrete,r=a.type===C.KEY&&a.modifier===M,i=n?"click":"turn",o=r?je(a.keycode):null,c=!!o,u=c?Je(o.plugin):null,p=Ca(u,i),h=c&&!n&&!!Number(o.value),v=!n&&a.type===C.CONSUMER?ar(a.keycode):null,y=h||!!v,m=h?rd(e,o):v?id(e,a):!1,k=Ac.filter(L=>!(n&&L.turnOnly)),N=U(),H=!!(N&&he(N,"rotary",pe(e,d.layer))),we=O().rotary?.[pe(e,d.layer)]||{type:0,modifier:0,keycode:0},ls=!n&&vn(a)?"Giro":t?.part.label;return`
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${x(t?.group.name||"Mando")}</h3>
        <span class="pill">${d.layer==="super"?"Capa SUPER":"Capa normal"}</span>
      </div>
      <p class="setting-desc inspector-sub">${x(ls||"")}</p>

      ${N?`
        <div class="override-note ${H?"is-override":""}">
          <span>${H?`Cambiado en <strong>${x(N.name)}</strong> · el perfil base hace
               <code>${x(Bt(we))}</code>`:`Editando <strong>${x(N.name)}</strong>: solo valdrá con esa app`}</span>
          ${H?`<button class="secondary-btn" data-act="clear-override">
                         ${f("reset",14)} Volver al valor base
                       </button>`:""}
        </div>`:""}

      <div class="field">
        <span class="field-label">Tipo de acción</span>
        <div class="type-grid">
          ${k.map(L=>`
            <button class="type-chip ${a.type===L.type&&!(L.type===C.KEY&&r)?"on":""}"
                    data-act="rotary-type" data-type="${L.type}">${L.label}</button>`).join("")}
          <button class="type-chip ${r&&!c?"on":""}" data-act="rotary-macro">Secuencia</button>
          ${Gi(i).map(L=>`
            <button class="type-chip ${u?.id===L.id?"on":""}"
                    data-act="rotary-plugin-tab" data-plugin="${L.id}">${x(L.name)}</button>`).join("")}
        </div>
      </div>

      ${c?`
        <div class="field">
          <span class="field-label">Qué controla</span>
          ${p.length?`
            <div class="consumer-grid">
              ${p.map(L=>`
                <button class="consumer-chip ${o.op===L.op?"on":""}"
                        data-act="${L.value?"plugin-value-open":"rotary-plugin"}" data-mode="rotary"
                        data-plugin="${o.plugin}"
                        data-op="${x(L.op)}">${x(L.label)}</button>`).join("")}
            </div>
            ${ee.open&&ee.mode==="rotary"&&ee.plugin===o.plugin?Oo(u,ee.op):""}`:`
            <p class="setting-desc">
              ${u?"Este complemento no ofrece nada para este mando.":`El complemento «${x(o.plugin)}» no está instalado o está
                   desactivado: el mando no hará nada hasta que vuelva.`}
            </p>`}
          ${y?`
            <button class="consumer-chip ${m?"on":""}" data-act="rotary-invert"
                    style="margin-top:8px">
              ${f("reset",14)} Invertir giro
            </button>`:""}
          <p class="setting-desc">
            ${n?"Esto lo maneja el PC, así que OrbyGUI tiene que estar abierto.":`Cada muesca mueve el valor un paso, y el sentido lo pone el propio giro: pon lo
                 mismo en los dos sentidos del encoder y ya sube y baja. Si el mando está montado
                 al revés, <em>Invertir giro</em> le da la vuelta a los dos sentidos a la vez.
                 Esto lo maneja el PC, así que OrbyGUI tiene que estar abierto.`}
          </p>
        </div>`:""}

      ${a.type===C.CONSUMER?`
        <div class="field">
          <span class="field-label">Acción</span>
          <div class="consumer-grid">
            ${n?Yt.map(L=>`
                  <button class="consumer-chip ${a.keycode===L.index?"on":""}"
                          data-act="rotary-consumer" data-index="${L.index}">${L.label}</button>`).join(""):oc.map(L=>L.pairId?`<button class="consumer-chip ${a.keycode===L.up||a.keycode===L.down?"on":""}"
                             data-act="rotary-consumer-pair" data-pair="${L.pairId}">${L.label}</button>`:`<button class="consumer-chip ${a.keycode===L.index?"on":""}"
                             data-act="rotary-consumer" data-index="${L.index}">${L.label}</button>`).join("")}
          </div>
          ${v?`
            <button class="consumer-chip ${m?"on":""}" data-act="rotary-invert"
                    style="margin-top:8px">
              ${f("reset",14)} Invertir giro
            </button>`:""}
          ${n?"":`
            <p class="setting-desc">
              Cada muesca sube o baja un paso, y el sentido lo pone el propio giro: no hace
              falta elegir qué va en cada lado. Si el mando está montado al revés,
              <em>Invertir giro</em> le da la vuelta a los dos sentidos a la vez.
            </p>`}
        </div>`:""}

      ${r&&!c?Po(a.keycode):""}

      ${a.type===C.KEY&&!r?`
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${Aa.map(L=>`
              <button class="mod-chip ${a.modifier&L.bit?"on":""}"
                      data-act="rotary-mod" data-bit="${L.bit}">${L.label}</button>`).join("")}
          </div>
        </div>
        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="rotary-keycode">
            <option value="0" ${a.keycode?"":"selected"}>— ninguna —</option>
            ${dr(a.keycode)}
          </select>
        </label>
        <button class="primary-btn full ${d.capturing?"is-capturing":""}" data-act="capture">
          ${f("key",16)} ${d.capturing?"Pulsa el atajo… (Esc cancela)":"Capturar atajo del teclado"}
        </button>`:""}

      ${nr(a.type)?`
        <p class="setting-desc">
          El sentido va implícito en el giro, así que esta acción cubre las dos direcciones.
          ${a.type===C.SCROLL_V?"En la rueda magnética es la única opción que aprovecha la alta resolución; el resto trabajan por clics completos.":""}
        </p>`:""}

      <div class="inspector-summary">
        <span class="field-label">Resultado</span>
        <code>${x(Bt(a))}</code>
      </div>
    </div>`}function Fd(){const e=Re(),t=O(),a=U(),n=F(e,d.layer),r=J(),i=r.modifier===gt,o=r.modifier===M,c=i||o||r.modifier===et||r.modifier===tt,u=dr(c?0:r.keycode),p=n>=0&&!!ye(d.editingProfile,n),h=t.keys[le(e,d.layer)]||{modifier:0,keycode:0},v=!!(a&&(he(a,"keys",le(e,d.layer))||he(a,"labels",n))),y=d.tab;return`
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>Tecla ${e+1}</h3>
        <div class="row-inline" style="gap:6px">
          <button class="tool-btn small" data-act="copy-key" title="Copiar icono, etiqueta y acción de esta tecla">
            ${f("copy",15)}
          </button>
          <button class="tool-btn small" data-act="paste-key" title="${de?"Pegar en esta tecla":"Copia una tecla primero"}"
                  ${de?"":"disabled"}>
            ${f("paste",15)}
          </button>
          <button class="tool-btn small" data-act="clear-action" title="Quita el atajo o la macro y deja la pantalla de la tecla en negro">
            ${f("trash",15)}
          </button>
          <span class="pill">${d.layer==="super"?"Capa SUPER":"Capa normal"}</span>
        </div>
      </div>

      ${a?`
        <div class="override-note ${v?"is-override":""}">
          <span>${v?`Cambiada en <strong>${x(a.name)}</strong> · el perfil base hace
               <code>${x(_t(h.modifier,h.keycode))}</code>`:`Editando <strong>${x(a.name)}</strong>: lo que cambies aquí solo
               valdrá con esa app`}</span>
          ${v?`<button class="secondary-btn" data-act="clear-override">
                         ${f("reset",14)} Volver al valor base
                       </button>`:""}
        </div>`:""}

      ${n>=0?`
        <div class="field">
          <span class="field-label">Pantalla OLED</span>
          <div class="inspector-screen">
            <span class="okey-screen">
              ${p?`<canvas class="okey-canvas" data-bmp="${ue(d.editingProfile,n)}"></canvas>`:`<span class="okey-text">${x(t.labels[n]||"—")}</span>`}
            </span>
            <button class="secondary-btn" data-act="edit-icon" data-key="${e}">
              ${f("pencil",16)} ${p?"Editar icono":"Dibujar icono"}
            </button>
          </div>
        </div>`:`
        <p class="setting-desc">
          Esta tecla no tiene pantalla: la ${ht} es el modificador SUPER y la ${tn}
          abre el menú del teclado al mantenerla, así que el firmware no ejecuta su atajo.
        </p>`}

      <div class="inspector-tabs">
        ${[{id:"shortcut",label:"Atajo",cap:null},{id:"sequence",label:"Secuencia",cap:null},{id:"text",label:"Texto",cap:"text"},{id:"record",label:"Grabar",cap:"recorder"},{id:"app",label:"App",cap:"openApp"},{id:"media",label:"Multimedia",cap:null}].map(({id:m,label:k,cap:N})=>{const H=N&&!V(N);return`<button class="inspector-tab ${y===m?"active":""} ${H?"unsupported":""}"
                          ${H?'disabled title="Necesita OrbyGUI de escritorio"':""}
                          data-act="set-tab" data-tab="${m}">${k}</button>`}).join("")}
        ${rt()?`<button class="inspector-tab ${y==="pages"?"active":""}" data-act="set-tab" data-tab="pages">Páginas</button>`:""}
      </div>

      ${y==="shortcut"?`
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${Aa.map(m=>`
              <button class="mod-chip ${!c&&r.modifier&m.bit?"on":""}"
                      data-act="toggle-mod" data-bit="${m.bit}">${m.label}</button>`).join("")}
          </div>
        </div>

        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="pick-keycode">
            <option value="0" ${!c&&!r.keycode?"selected":""}>— ninguna —</option>
            ${u}
          </select>
        </label>

        <div class="inspector-actions">
          <button class="primary-btn full ${d.capturing===!0?"is-capturing":""}" data-act="capture">
            ${f("key",16)} ${d.capturing===!0?"Pulsa el atajo… (Esc cancela)":"Capturar atajo del teclado"}
          </button>
        </div>`:""}

      ${y==="sequence"?Po(o?r.keycode:null):""}

      ${y==="text"?pd(o?r.keycode:null):""}

      ${y==="record"?Xc(o?r.keycode:null):""}

      ${y==="app"?ud(o?r.keycode:null):""}

      ${y==="media"?`
        <div class="field">
          <span class="field-label">Acción multimedia</span>
          <div class="consumer-grid consumer-grid-3col">
            ${Yt.map(m=>`
              <button class="consumer-chip ${i&&r.keycode===m.index?"on":""}"
                      data-act="set-consumer" data-index="${m.index}">${m.label}</button>`).join("")}
          </div>
        </div>
        <div class="field">
          <span class="field-label">Energía del PC</span>
          <div class="consumer-grid">
            ${Object.entries(ao).map(([m,k])=>`
              <button class="consumer-chip ${o&&Qt(r.keycode)&&I(r.keycode).actions[0].mode===m?"on":""}"
                      data-act="set-power" data-mode="${m}">${k}</button>`).join("")}
          </div>
          <p class="setting-desc">
            Esta acción la ejecuta el PC, no el teclado, así que OrbyGUI tiene que estar abierto
            —vale con el icono de la bandeja— para que funcione.
          </p>
        </div>
        ${Wd(o?r.keycode:null)}`:""}

      ${y==="pages"?Gd(r):""}
    </div>`}function Wd(e){const t=e===null?null:je(e),a=ee;return Gi("key").map(n=>`
    <div class="field">
      <span class="field-label">${x(n.name)}</span>
      <div class="consumer-grid">
        ${Ca(n,"key").map(r=>`
          <button class="consumer-chip ${t?.plugin===n.id&&t.op===r.op?"on":""}"
                  data-act="${r.value?"plugin-value-open":"set-plugin"}" data-mode="key"
                  data-plugin="${n.id}" data-op="${x(r.op)}">
            ${x(r.label)}
          </button>`).join("")}
      </div>
      ${a.open&&a.mode==="key"&&a.plugin===n.id?Oo(n,a.op):""}
      <p class="setting-desc">
        ${x(n.description||"Lo ejecuta el PC, así que OrbyGUI tiene que estar abierto.")}
        Lo que necesite sentido de giro (subir, bajar) se asigna a un mando, no a una tecla.
      </p>
    </div>`).join("")+Ud(e)}function Oo(e,t){const{action:a}=Sa(e.id,t);if(!a?.value)return"";const{min:n,max:r,step:i}=a.value,o=ee.value;return`
    <div class="plugin-value-pick">
      <div class="field">
        <span class="field-label">${x(a.label)} <b id="plugin-value-readout">${o}</b></span>
        <input type="range" class="premium-slider" data-act="plugin-value-slider"
               min="${n}" max="${r}" step="${i}" value="${o}">
      </div>
      <div class="row-inline" style="gap:6px">
        <button class="primary-btn" data-act="plugin-value-confirm">${f("check",16)} Aplicar</button>
        <button class="secondary-btn" data-act="plugin-value-cancel">${f("close",16)} Cancelar</button>
      </div>
    </div>`}function Ud(e){const t=e===null?null:je(e);if(!(ze[Re()]>0))return"";const n=Yl().map(r=>`
    <div class="field">
      <span class="field-label">${x(r.name)} · pantalla</span>
      <div class="consumer-grid">
        ${Ia(r).map(i=>`
          <button class="consumer-chip ${t?.plugin===r.id&&t.op===i.op?"on":""}"
                  data-act="set-plugin" data-plugin="${r.id}" data-op="${x(i.op)}">
            ${x(i.label)}
          </button>`).join("")}
      </div>
    </div>`).join("");return n?`${n}
    <p class="setting-desc">
      La tecla deja de hacer nada al pulsarla: solo enseña el valor en su pantalla, y se
      actualiza solo cada par de segundos mientras esta página esté puesta en el teclado.
    </p>`:""}function Gd(e){if(!rt())return"";const t=s.profiles[d.editingProfile],a=t?D(t):1,n=e.modifier===et,r=e.modifier===tt;let i="";for(let o=1;o<=Ut();o++){const c=o>a;i+=`<button class="consumer-chip ${n&&e.keycode===o?"on":""}"
                      data-act="set-goto-page" data-page="${o}"
                      title="${c?"Este perfil todavía no tiene esa página":""}">
                Página ${o}${c?" ·":""}
              </button>`}return`
    <div class="field">
      <span class="field-label">Páginas</span>
      <div class="consumer-grid">${i}</div>
      <button class="consumer-chip full ${r?"on":""}" data-act="set-page-state">
        Estado de páginas
      </button>
      <p class="setting-desc">
        <strong>Página N</strong> salta directamente a esa página.
        <strong>Estado de páginas</strong> hace que la tecla enseñe en qué página estás
        (por ejemplo <code>P2/${a}</code>) y que al pulsarla se abra el gestor: cada
        pantalla con su número, la actual en negativo, y pulsas para saltar.
      </p>
    </div>`}const Kd=Object.freeze(Object.defineProperty({__proto__:null,init:Co,render:P,syncAllMacrosToDevice:io},Symbol.toStringTag,{value:"Module"})),b='fill="currentColor" stroke="none"',Xd=[{id:"basic",label:"Formas y flechas"},{id:"media",label:"Multimedia"},{id:"edit",label:"Edición"},{id:"system",label:"Sistema"},{id:"dev",label:"Programación"},{id:"comm",label:"Comunicación"},{id:"misc",label:"Varios"}],Yd={basic:[["arrow-up","Flecha arriba","subir norte",'<path d="M12 20V4M5 11l7-7 7 7"/>'],["arrow-down","Flecha abajo","bajar sur",'<path d="M12 4v16M5 13l7 7 7-7"/>'],["arrow-left","Flecha izquierda","atras oeste",'<path d="M20 12H4M11 5l-7 7 7 7"/>'],["arrow-right","Flecha derecha","siguiente este",'<path d="M4 12h16M13 5l7 7-7 7"/>'],["arrow-ul","Flecha arriba izquierda","diagonal",'<path d="M18 18L6 6M6 14V6h8"/>'],["arrow-ur","Flecha arriba derecha","diagonal",'<path d="M6 18L18 6M18 14V6h-8"/>'],["arrow-dl","Flecha abajo izquierda","diagonal",'<path d="M18 6L6 18M6 10v8h8"/>'],["arrow-dr","Flecha abajo derecha","diagonal",'<path d="M6 6l12 12M18 10v8h-8"/>'],["chevron-up","Punta arriba","desplegar",'<path d="M5 15l7-7 7 7"/>'],["chevron-down","Punta abajo","desplegar menu",'<path d="M5 9l7 7 7-7"/>'],["chevron-left","Punta izquierda","anterior",'<path d="M15 5l-7 7 7 7"/>'],["chevron-right","Punta derecha","siguiente",'<path d="M9 5l7 7-7 7"/>'],["chevrons-left","Doble punta izquierda","inicio retroceder",'<path d="M11 18l-6-6 6-6M19 18l-6-6 6-6"/>'],["chevrons-right","Doble punta derecha","fin avanzar",'<path d="M13 6l6 6-6 6M5 6l6 6-6 6"/>'],["arrows-h","Flechas horizontales","ancho ajustar",'<path d="M3 12h18M6.5 8.5L3 12l3.5 3.5M17.5 8.5L21 12l-3.5 3.5"/>'],["arrows-v","Flechas verticales","alto ajustar",'<path d="M12 3v18M8.5 6.5L12 3l3.5 3.5M8.5 17.5L12 21l3.5-3.5"/>'],["rotate-cw","Girar a la derecha","rehacer recargar",'<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>'],["rotate-ccw","Girar a la izquierda","deshacer recargar",'<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>'],["shuffle","Aleatorio","mezclar random",'<path d="M3 6h4l10 12h4M3 18h4l2.5-3M14.5 8.5L17 6h4"/><path d="M18 3l3 3-3 3M18 15l3 3-3 3"/>'],["repeat","Repetir","bucle loop",'<path d="M4 10V8a3 3 0 0 1 3-3h13"/><path d="M17 2l3 3-3 3"/><path d="M20 14v2a3 3 0 0 1-3 3H4"/><path d="M7 22l-3-3 3-3"/>'],["plus","Más","añadir sumar nuevo",'<path d="M12 4v16M4 12h16"/>'],["minus","Menos","quitar restar",'<path d="M4 12h16"/>'],["close","Cerrar","equis cancelar salir",'<path d="M5 5l14 14M19 5L5 19"/>'],["check","Confirmar","ok aceptar tick visto",'<path d="M4 12.5l5.5 5.5L20 6"/>'],["check-circle","Confirmado","ok aceptar",'<circle cx="12" cy="12" r="9"/><path d="M7.5 12.5l3 3 6-6.5"/>'],["x-circle","Cancelar","error borrar",'<circle cx="12" cy="12" r="9"/><path d="M8.5 8.5l7 7M15.5 8.5l-7 7"/>'],["circle","Círculo","redondo",'<circle cx="12" cy="12" r="8"/>'],["circle-fill","Círculo macizo","punto bola",`<circle cx="12" cy="12" r="8" ${b}/>`],["square","Cuadrado","caja",'<rect x="4" y="4" width="16" height="16" rx="2"/>'],["square-fill","Cuadrado macizo","caja bloque",`<rect x="4" y="4" width="16" height="16" rx="2" ${b}/>`],["triangle","Triángulo","aviso",'<path d="M12 4l9 16H3z"/>'],["triangle-fill","Triángulo macizo","aviso",`<path d="M12 4l9 16H3z" ${b}/>`],["star","Estrella","favorito destacar",'<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z"/>'],["star-fill","Estrella maciza","favorito destacar",`<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z" ${b}/>`],["heart","Corazón","favorito megusta",'<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z"/>'],["heart-fill","Corazón macizo","favorito megusta",`<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z" ${b}/>`],["diamond","Rombo","diamante",'<path d="M12 3l9 9-9 9-9-9z"/>'],["hexagon","Hexágono","panal",'<path d="M12 2.5l8 4.6v9.8l-8 4.6-8-4.6V7.1z"/>']],media:[["play","Reproducir","play iniciar",`<path d="M7 4.5l12 7.5-12 7.5z" ${b}/>`],["play-circle","Reproducir en círculo","play",`<circle cx="12" cy="12" r="9"/><path d="M10 8.5l6 3.5-6 3.5z" ${b}/>`],["pause","Pausa","pausar",`<rect x="6" y="4" width="4" height="16" rx="1" ${b}/><rect x="14" y="4" width="4" height="16" rx="1" ${b}/>`],["pause-circle","Pausa en círculo","pausar",'<circle cx="12" cy="12" r="9"/><path d="M10 8v8M14 8v8"/>'],["stop","Parar","detener stop",`<rect x="5" y="5" width="14" height="14" rx="2" ${b}/>`],["record","Grabar","rec grabacion",`<circle cx="12" cy="12" r="7" ${b}/>`],["skip-next","Siguiente pista","next adelante",`<path d="M5 4.5l11 7.5-11 7.5z" ${b}/><rect x="17" y="4.5" width="3" height="15" rx="1" ${b}/>`],["skip-prev","Pista anterior","prev atras",`<path d="M19 4.5v15L8 12z" ${b}/><rect x="4" y="4.5" width="3" height="15" rx="1" ${b}/>`],["fast-forward","Avance rápido","ff acelerar",`<path d="M3 5l9 7-9 7z" ${b}/><path d="M13 5l9 7-9 7z" ${b}/>`],["rewind","Retroceso rápido","rw rebobinar",`<path d="M21 5l-9 7 9 7z" ${b}/><path d="M11 5l-9 7 9 7z" ${b}/>`],["volume-high","Volumen alto","sonido subir altavoz",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 8.5a5 5 0 0 1 0 7M19.5 6a9 9 0 0 1 0 12"/>`],["volume-low","Volumen bajo","sonido bajar altavoz",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 8.5a5 5 0 0 1 0 7"/>`],["volume-mute","Silencio","mute sonido apagado",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 9l5 6M21.5 9l-5 6"/>`],["mic","Micrófono","grabar voz",'<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/>'],["mic-off","Micrófono apagado","silenciar mute voz",'<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/><path d="M3 3l18 18"/>'],["headphones","Auriculares","cascos audio",'<path d="M4 15v-3a8 8 0 0 1 16 0v3"/><rect x="2" y="14" width="5" height="7" rx="2"/><rect x="17" y="14" width="5" height="7" rx="2"/>'],["music","Música","nota cancion",'<path d="M9 18V5l11-2v13"/><circle cx="6" cy="18" r="3"/><circle cx="17" cy="16" r="3"/>'],["playlist","Lista de reproducción","cola",'<path d="M3 6h12M3 11h12M3 16h7"/><circle cx="17" cy="17.5" r="2.5"/><path d="M19.5 17.5V9l2.5 1"/>'],["camera","Cámara","foto captura",'<path d="M3 9a2 2 0 0 1 2-2h2l1.5-2.5h7L17 7h2a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><circle cx="12" cy="13" r="3.5"/>'],["video","Vídeo","camara grabar",'<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/>'],["video-off","Vídeo apagado","camara cortar",'<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/><path d="M3 3l18 18"/>'],["image","Imagen","foto galeria",'<rect x="3" y="4" width="18" height="16" rx="2"/><circle cx="8.5" cy="9.5" r="1.8"/><path d="M4 18l5-5 3.5 3.5L16 13l4 4"/>'],["film","Película","cine video",'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M7 4v16M17 4v16M2 9h5M2 15h5M17 9h5M17 15h5"/>'],["tv","Televisión","pantalla monitor",'<rect x="2" y="6" width="20" height="13" rx="2"/><path d="M8 2l4 4 4-4"/>'],["cast","Enviar a pantalla","streaming airplay",'<path d="M2 16.5a5.5 5.5 0 0 1 5.5 5.5M2 12a10 10 0 0 1 10 10"/><path d="M2 8V6a2 2 0 0 1 2-2h16a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2h-5"/>'],["eject","Expulsar","sacar disco",`<path d="M12 4l8 10H4z" ${b}/><rect x="4" y="17" width="16" height="3" rx="1" ${b}/>`]],edit:[["copy","Copiar","duplicar ctrlc",'<rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>'],["paste","Pegar","ctrlv portapapeles",'<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>'],["scissors","Cortar","tijeras ctrlx",'<circle cx="6" cy="6" r="2.5"/><circle cx="6" cy="18" r="2.5"/><path d="M8 7.5L20 19M20 5L8 16.5"/>'],["undo","Deshacer","ctrlz atras",'<path d="M4 12h11a5 5 0 0 1 0 10h-3"/><path d="M8 8l-4 4 4 4"/>'],["redo","Rehacer","ctrly adelante",'<path d="M20 12H9a5 5 0 0 0 0 10h3"/><path d="M16 8l4 4-4 4"/>'],["save","Guardar","ctrls disquete",'<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>'],["trash","Borrar","papelera eliminar",'<path d="M3 6h18M8 6V4h8v2"/><path d="M6 6l1 14h10l1-14"/><path d="M10 10v7M14 10v7"/>'],["pencil","Lápiz","editar escribir",'<path d="M17 3l4 4L8 20l-5 1 1-5z"/><path d="M14.5 5.5l4 4"/>'],["brush","Pincel","pintar dibujar",'<path d="M9 14l-3 3c-1 1-1 3 0 4s3 1 4 0l3-3z"/><path d="M11 12l8-8 3 3-8 8z"/>'],["eraser","Borrador","goma",'<path d="M8.5 20.5l-5-5a2 2 0 0 1 0-2.8L13.6 2.6a2 2 0 0 1 2.8 0l4.5 4.5a2 2 0 0 1 0 2.8L11 20.5z"/><path d="M9 20.5h11M8.5 8l7 7"/>'],["text","Texto","letra fuente tipografia",'<path d="M4 7V4h16v3M12 4v16M8 20h8"/>'],["bold","Negrita","grueso",'<path d="M7 4h6a4 4 0 0 1 0 8H7zM7 12h7a4 4 0 0 1 0 8H7z"/>'],["italic","Cursiva","inclinada",'<path d="M10 4h8M6 20h8M14.5 4l-5 16"/>'],["underline","Subrayado","linea",'<path d="M7 4v7a5 5 0 0 0 10 0V4M5 20h14"/>'],["align-left","Alinear a la izquierda","parrafo",'<path d="M3 5h18M3 10h11M3 15h15M3 20h8"/>'],["align-center","Centrar","parrafo",'<path d="M3 5h18M6.5 10h11M4 15h16M8 20h8"/>'],["align-right","Alinear a la derecha","parrafo",'<path d="M3 5h18M10 10h11M6 15h15M13 20h8"/>'],["list-ul","Lista","viñetas puntos",`<path d="M9 6h12M9 12h12M9 18h12"/><circle cx="4.5" cy="6" r="1.4" ${b}/><circle cx="4.5" cy="12" r="1.4" ${b}/><circle cx="4.5" cy="18" r="1.4" ${b}/>`],["list-ol","Lista numerada","orden",'<path d="M9 6h12M9 12h12M9 18h12"/><path d="M3 4.5h1.5V9M3 10.5h2.5L3 14.5h2.5M3 16h2.5v2H3.5v2H6"/>'],["link","Enlace","url vinculo",'<path d="M10.5 13.5a4.5 4.5 0 0 0 6.4 0l2.5-2.5a4.5 4.5 0 0 0-6.4-6.4l-1.4 1.4"/><path d="M13.5 10.5a4.5 4.5 0 0 0-6.4 0l-2.5 2.5a4.5 4.5 0 0 0 6.4 6.4l1.4-1.4"/>'],["unlink","Quitar enlace","romper vinculo",'<path d="M9 15l-1.5 1.5a4.5 4.5 0 0 1-6.4-6.4L4 7.5"/><path d="M15 9l1.5-1.5a4.5 4.5 0 0 1 6.4 6.4L20 16.5"/><path d="M3 3l18 18"/>'],["crop","Recortar","encuadre",'<path d="M6 2v16h16M2 6h16v16"/>'],["layers","Capas","niveles",'<path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/>'],["palette","Paleta","color pintura",`<path d="M12 3a9 9 0 0 0 0 18c1.5 0 2-1 1.5-2s0-2 1.5-2H18a3 3 0 0 0 3-3c0-5-4-9-9-9z"/><circle cx="7.5" cy="12.5" r="1.3" ${b}/><circle cx="9.5" cy="8.5" r="1.3" ${b}/><circle cx="14" cy="7.5" r="1.3" ${b}/>`],["zoom-in","Acercar","lupa aumentar zoom",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5M10.5 8v5"/>'],["zoom-out","Alejar","lupa reducir zoom",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5"/>'],["grid","Rejilla","cuadricula tabla",'<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>'],["move","Mover","desplazar arrastrar",'<path d="M12 3v18M3 12h18"/><path d="M9 6l3-3 3 3M9 18l3 3 3-3M6 9l-3 3 3 3M18 9l3 3-3 3"/>'],["magic-wand","Varita mágica","automatico efecto",'<path d="M15 4.5l4.5 4.5L9 19.5 4.5 15z"/><path d="M12.5 7l4.5 4.5"/><path d="M19 2v3M22.5 4.5h-3M21 9l1.5.8"/>'],["pipette","Cuentagotas","color muestra",'<path d="M18 2l4 4-2.5 2.5-1-1L8 20l-4 1 1-4L16.5 5.5l-1-1z"/>']],system:[["power","Encendido","apagar boton power",'<path d="M12 3v9"/><path d="M7.5 6.5a7 7 0 1 0 9 0"/>'],["gear","Ajustes","configuracion opciones tuerca",'<path d="M10.3 3h3.4l.4 2.3 1.9.8 1.9-1.3 2.4 2.4-1.3 1.9.8 1.9 2.3.4v3.4l-2.3.4-.8 1.9 1.3 1.9-2.4 2.4-1.9-1.3-1.9.8-.4 2.3h-3.4l-.4-2.3-1.9-.8-1.9 1.3-2.4-2.4 1.3-1.9-.8-1.9L2.5 13.7v-3.4l2.3-.4.8-1.9-1.3-1.9 2.4-2.4 1.9 1.3 1.9-.8z"/><circle cx="12" cy="12" r="3"/>'],["sliders","Controles","ecualizador ajustes",'<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2.2"/><circle cx="15" cy="12" r="2.2"/><circle cx="8" cy="18" r="2.2"/>'],["home","Inicio","casa principal",'<path d="M3 11l9-7 9 7"/><path d="M5.5 9.5V20h13V9.5"/><path d="M10 20v-5h4v5"/>'],["search","Buscar","lupa encontrar",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21"/>'],["lock","Bloqueado","candado seguridad",'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/>'],["unlock","Desbloqueado","candado abierto",'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 7.6-2"/>'],["key","Llave","clave contraseña",'<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>'],["user","Usuario","perfil persona cuenta",'<circle cx="12" cy="8" r="4"/><path d="M4 21c0-4.4 3.6-7 8-7s8 2.6 8 7"/>'],["users","Usuarios","grupo equipo",'<circle cx="9" cy="8" r="3.5"/><path d="M2 21c0-4 3.1-6.5 7-6.5s7 2.5 7 6.5"/><path d="M16 5.3a3.5 3.5 0 0 1 0 6.9M18 14.4c2.4.8 4 2.8 4 5.6"/>'],["bell","Aviso","notificacion campana alerta",'<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/>'],["bell-off","Avisos apagados","silenciar notificacion",'<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/><path d="M3 3l18 18"/>'],["wifi","Wi-Fi","red inalambrica",`<path d="M2.5 8.5a15 15 0 0 1 19 0"/><path d="M5.8 12.2a10 10 0 0 1 12.4 0"/><path d="M9 15.9a5 5 0 0 1 6 0"/><circle cx="12" cy="19.5" r="1.4" ${b}/>`],["wifi-off","Sin Wi-Fi","red caida",`<path d="M5.8 12.2a10 10 0 0 1 8-2.1"/><path d="M9 15.9a5 5 0 0 1 4-.8"/><circle cx="12" cy="19.5" r="1.4" ${b}/><path d="M3 3l18 18"/>`],["bluetooth","Bluetooth","emparejar",'<path d="M7 7l10 10-5 4V3l5 4L7 17"/>'],["battery","Batería","pila carga",`<rect x="2" y="7" width="17" height="10" rx="2.5"/><path d="M21.5 10.5v3"/><rect x="4.5" y="9.5" width="7" height="5" rx="1" ${b}/>`],["battery-charging","Cargando","bateria enchufe",'<path d="M14 7h3a2.5 2.5 0 0 1 2 2v6a2.5 2.5 0 0 1-2 2h-4"/><path d="M8 7H4.5a2.5 2.5 0 0 0-2.5 2.5v5A2.5 2.5 0 0 0 4.5 17H8"/><path d="M22 10.5v3"/><path d="M12 5.5L8.5 12H12l-1 6.5 4.5-7H12z"/>'],["usb","USB","conector cable",`<circle cx="12" cy="20.5" r="1.6" ${b}/><path d="M12 19V4"/><path d="M9 7l3-4 3 4"/><path d="M12 13l4-3V7.5"/><circle cx="16" cy="6.5" r="1.6" ${b}/><path d="M12 16l-4-3V10"/><rect x="6.5" y="7.5" width="3" height="2.5" ${b}/>`],["monitor","Monitor","pantalla ordenador",'<rect x="2" y="4" width="20" height="13" rx="2"/><path d="M8 21h8M12 17v4"/>'],["laptop","Portátil","ordenador",'<rect x="4" y="5" width="16" height="11" rx="2"/><path d="M2 19h20"/>'],["keyboard","Teclado","teclas escribir",'<rect x="2" y="6" width="20" height="12" rx="2"/><path d="M6 10h.01M10 10h.01M14 10h.01M18 10h.01M8 14h8"/>'],["mouse","Ratón","puntero clic",'<rect x="7" y="3" width="10" height="18" rx="5"/><path d="M12 7v3.5"/>'],["cpu","Procesador","chip micro",`<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9.5" y="9.5" width="5" height="5" rx="1" ${b}/><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3"/>`],["hdd","Disco","almacenamiento unidad",`<rect x="2" y="9" width="20" height="10" rx="2"/><path d="M5 4h14l3 5H2z"/><circle cx="17.5" cy="14" r="1.4" ${b}/>`],["printer","Impresora","imprimir",'<path d="M6 9V3h12v6"/><rect x="2" y="9" width="20" height="8" rx="2"/><path d="M6 14h12v7H6z"/>'],["calculator","Calculadora","numeros cuentas",'<rect x="4" y="2" width="16" height="20" rx="2"/><rect x="7" y="5" width="10" height="4" rx="1"/><path d="M8 13h.01M12 13h.01M16 13h.01M8 17.5h.01M12 17.5h.01M16 17.5h.01"/>'],["clock","Reloj","hora tiempo",'<circle cx="12" cy="12" r="9"/><path d="M12 6.5V12l3.5 2"/>'],["alarm","Alarma","despertador aviso",'<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4l3 2"/><path d="M5.5 3L2.5 6M18.5 3l3 3"/>'],["calendar","Calendario","fecha agenda",'<rect x="3" y="5" width="18" height="16" rx="2"/><path d="M3 10h18M8 3v4M16 3v4"/>'],["folder","Carpeta","directorio archivos",'<path d="M3 7a2 2 0 0 1 2-2h4l2 2.5h8a2 2 0 0 1 2 2V18a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>'],["folder-open","Carpeta abierta","directorio",'<path d="M3 8a2 2 0 0 1 2-2h4l2 2.5h6a2 2 0 0 1 2 2V12"/><path d="M3 20l2.6-7.3a1.5 1.5 0 0 1 1.4-1H20.5a1 1 0 0 1 .95 1.3L19 20z"/>'],["file","Archivo","documento fichero",'<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5"/>'],["file-text","Documento","texto informe",'<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5M9 13h6M9 17h4"/>'],["files","Archivos","documentos varios",'<rect x="8" y="3" width="12" height="15" rx="2"/><path d="M16 21H6a2 2 0 0 1-2-2V7"/>'],["download","Descargar","bajar guardar",'<path d="M12 3v12M7 11l5 5 5-5"/><path d="M4 20h16"/>'],["upload","Subir","enviar cargar",'<path d="M12 21V9M7 13l5-5 5 5"/><path d="M4 4h16"/>'],["sync","Sincronizar","actualizar refrescar",'<path d="M20 11a8 8 0 0 0-13.7-5.2L4 8"/><path d="M4 3.5V8h4.5"/><path d="M4 13a8 8 0 0 0 13.7 5.2L20 16"/><path d="M20 20.5V16h-4.5"/>'],["cloud","Nube","internet online",'<path d="M6.8 19a4.5 4.5 0 0 1-.4-9 6 6 0 0 1 11.4 1 4.2 4.2 0 0 1-.6 8z"/>'],["cloud-up","Subir a la nube","backup copia",'<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 21v-8M9 15.5l3-3 3 3"/>'],["cloud-down","Bajar de la nube","restaurar descarga",'<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 13v8M9 18.5l3 3 3-3"/>'],["database","Base de datos","datos sql",'<ellipse cx="12" cy="6" rx="8" ry="3"/><path d="M4 6v12c0 1.7 3.6 3 8 3s8-1.3 8-3V6"/><path d="M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3"/>'],["server","Servidor","rack hosting",'<rect x="3" y="4" width="18" height="7" rx="2"/><rect x="3" y="13" width="18" height="7" rx="2"/><path d="M7 7.5h.01M7 16.5h.01"/>'],["network","Red","nodos conexion",'<rect x="9" y="2" width="6" height="6" rx="1"/><rect x="2" y="16" width="6" height="6" rx="1"/><rect x="16" y="16" width="6" height="6" rx="1"/><path d="M12 8v3M5 16v-3h14v3"/>'],["shield","Escudo","seguridad proteccion",'<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/>'],["shield-check","Protegido","seguridad ok antivirus",'<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/><path d="M8.5 12l2.5 2.5 4.5-5"/>'],["plug","Enchufe","conectar corriente",'<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>'],["menu","Menú","hamburguesa opciones",'<path d="M4 7h16M4 12h16M4 17h16"/>'],["more-h","Más opciones","puntos horizontal",`<circle cx="5" cy="12" r="1.8" ${b}/><circle cx="12" cy="12" r="1.8" ${b}/><circle cx="19" cy="12" r="1.8" ${b}/>`],["more-v","Más opciones vertical","puntos",`<circle cx="12" cy="5" r="1.8" ${b}/><circle cx="12" cy="12" r="1.8" ${b}/><circle cx="12" cy="19" r="1.8" ${b}/>`],["filter","Filtrar","embudo criterios",'<path d="M3 5h18l-7 8v6l-4 2v-8z"/>'],["sort","Ordenar","clasificar",'<path d="M3 7h10M3 12h7M3 17h4"/><path d="M17 5v14M14 16l3 3 3-3"/>'],["eye","Ver","ojo mostrar visible",'<path d="M2 12s3.6-6.5 10-6.5S22 12 22 12s-3.6 6.5-10 6.5S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>'],["eye-off","Ocultar","ojo invisible",'<path d="M4.5 7.5C3 9.2 2 12 2 12s3.6 6.5 10 6.5c2 0 3.7-.5 5.1-1.3M9.5 5.8A11 11 0 0 1 12 5.5c6.4 0 10 6.5 10 6.5s-1 1.8-2.7 3.5"/><path d="M9.9 9.9a3 3 0 0 0 4.2 4.2"/><path d="M3 3l18 18"/>'],["maximize","Maximizar","pantalla completa",'<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/>'],["minimize","Minimizar","reducir ventana",'<path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5"/>'],["external-link","Abrir fuera","enlace externo ventana",'<path d="M14 4h6v6M20 4L10 14"/><path d="M18 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7a1 1 0 0 1 1-1h5"/>'],["window","Ventana","app programa",'<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 9h18M6.5 6.5h.01M9.5 6.5h.01"/>']],dev:[["code","Código","programar corchetes",'<path d="M9 18l-6-6 6-6M15 6l6 6-6 6"/>'],["braces","Llaves","json bloque",'<path d="M9 3c-2 0-3 1-3 3v2c0 2-1 3-2 3 1 0 2 1 2 3v2c0 2 1 3 3 3"/><path d="M15 3c2 0 3 1 3 3v2c0 2 1 3 2 3-1 0-2 1-2 3v2c0 2-1 3-3 3"/>'],["terminal","Terminal","consola shell cmd",'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>'],["bug","Depurar","error bicho debug",'<rect x="8" y="7" width="8" height="12" rx="4"/><path d="M8 11H4M8 15.5H3.5M16 11h4M16 15.5h4.5M9.5 7.5L8 5M14.5 7.5L16 5M12 19v3"/>'],["git-branch","Rama","git branch",'<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="9" r="2.5"/><path d="M7 8.5v7M17 11.5c0 3.5-3.5 4-7 4.5"/>'],["git-commit","Commit","git guardar",'<circle cx="12" cy="12" r="3.2"/><path d="M2 12h6.8M15.2 12H22"/>'],["git-merge","Fusionar","git merge",'<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="13" r="2.5"/><path d="M7 8.5v7M14.5 13c-5 0-7.5-1.5-7.5-4.5"/>'],["package","Paquete","npm modulo caja",'<path d="M21 8l-9-5-9 5v8l9 5 9-5z"/><path d="M3 8l9 5 9-5M12 13v8"/>'],["box","Caja","contenedor almacen",'<rect x="3" y="7" width="18" height="13" rx="2"/><path d="M3 11h18M8 7V4h8v3"/>'],["robot","Robot","bot automatizar ia",`<rect x="4" y="8" width="16" height="12" rx="3"/><circle cx="9" cy="14" r="1.6" ${b}/><circle cx="15" cy="14" r="1.6" ${b}/><path d="M12 4v4M9 3.5h6"/>`],["activity","Actividad","pulso monitor rendimiento",'<path d="M3 12h4l3 8 4-16 3 8h4"/>'],["chart-bar","Gráfico de barras","estadisticas datos",'<path d="M3 21h18"/><path d="M6 21V11M12 21V4M18 21v-6"/>'],["chart-line","Gráfico de líneas","tendencia datos",'<path d="M3 3v18h18"/><path d="M6.5 15l4-4.5 3 3 6.5-7.5"/>'],["chart-pie","Gráfico circular","tarta porcentaje",'<path d="M12 3a9 9 0 1 0 9 9h-9z"/><path d="M14.5 2.3A9 9 0 0 1 21.7 9.5H14.5z"/>']],comm:[["mail","Correo","email sobre mensaje",'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M3 7l9 6 9-6"/>'],["mail-open","Correo abierto","email leido",'<path d="M3 10l9-6 9 6v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><path d="M3 10l9 6 9-6"/>'],["send","Enviar","mandar avion mensaje",'<path d="M21 3L10.5 13.5M21 3l-7 18-3.5-7.5L3 10z"/>'],["chat","Chat","mensaje conversacion",'<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/>'],["chat-dots","Mensaje","chat escribiendo",`<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/><circle cx="8" cy="10" r="1.2" ${b}/><circle cx="12" cy="10" r="1.2" ${b}/><circle cx="16" cy="10" r="1.2" ${b}/>`],["phone","Teléfono","llamar llamada",'<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/>'],["phone-off","Colgar","llamada cortar",'<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/><path d="M3 3l18 18"/>'],["at","Arroba","email usuario",'<circle cx="12" cy="12" r="4"/><path d="M16 8v5.5a2.5 2.5 0 0 0 5 0V12A9 9 0 1 0 17.4 19.2"/>'],["share","Compartir","enviar nodos",'<circle cx="18" cy="5" r="2.5"/><circle cx="6" cy="12" r="2.5"/><circle cx="18" cy="19" r="2.5"/><path d="M8.2 10.8l7.6-4.4M8.2 13.2l7.6 4.4"/>'],["megaphone","Megáfono","anuncio aviso",'<path d="M3 10.5v3l11 5V5.5z"/><path d="M14 9a3 3 0 0 1 0 6"/><path d="M6 15v4.5h3.5V16.5"/>'],["rss","RSS","feed suscripcion",`<circle cx="6" cy="18" r="1.9" ${b}/><path d="M4 11a9 9 0 0 1 9 9M4 4a16 16 0 0 1 16 16"/>`]],misc:[["flag","Bandera","marcar objetivo",'<path d="M5 21V4M5 5h13l-2.5 4.5L18 14H5"/>'],["pin","Marcador","ubicacion mapa chincheta",'<path d="M12 21.5S19 14.5 19 10a7 7 0 1 0-14 0c0 4.5 7 11.5 7 11.5z"/><circle cx="12" cy="10" r="2.5"/>'],["map","Mapa","ubicacion ruta",'<path d="M9 4L3 6.5v13L9 17l6 2.5 6-2.5v-13L15 6.5z"/><path d="M9 4v13M15 6.5v13"/>'],["globe","Mundo","internet web idioma",'<circle cx="12" cy="12" r="9"/><path d="M3 12h18"/><path d="M12 3a14 14 0 0 1 0 18 14 14 0 0 1 0-18z"/>'],["compass","Brújula","norte direccion",'<circle cx="12" cy="12" r="9"/><path d="M15.5 8.5l-2 5-5 2 2-5z"/>'],["sun","Sol","brillo claro dia",'<circle cx="12" cy="12" r="4"/><path d="M12 2v2.5M12 19.5V22M4.2 4.2l1.8 1.8M18 18l1.8 1.8M2 12h2.5M19.5 12H22M4.2 19.8L6 18M18 6l1.8-1.8"/>'],["moon","Luna","noche oscuro dormir",'<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>'],["flame","Fuego","llama caliente racha",'<path d="M12 2.5s5.5 4.8 5.5 9.5a5.5 5.5 0 0 1-11 0c0-2.2 1-3.9 2.2-5 0 2.2 1 3.2 2 3.2 1.6 0 1.1-4.3-1.2-8z"/>'],["drop","Gota","agua humedad",'<path d="M12 3.5s6 6.4 6 10.5a6 6 0 0 1-12 0C6 9.9 12 3.5 12 3.5z"/>'],["leaf","Hoja","planta eco natural",'<path d="M4 20c0-9 6-14 16-14 0 9-5 14-12 14H4z"/><path d="M9 15c2-3 5-5 8-6"/>'],["bolt","Rayo","energia rapido",'<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>'],["rocket","Cohete","lanzar rapido inicio",'<path d="M12 2c3.5 2.6 5.5 6.6 5.5 11l-2.5 3h-6l-2.5-3C6.5 8.6 8.5 4.6 12 2z"/><circle cx="12" cy="9.5" r="2"/><path d="M9 19l-2.5 3M15 19l2.5 3"/>'],["gift","Regalo","premio caja",'<rect x="3" y="8" width="18" height="4" rx="1"/><path d="M5 12v9h14v-9M12 8v13"/><path d="M12 8S10.5 4 8.5 4a2 2 0 0 0 0 4zM12 8s1.5-4 3.5-4a2 2 0 0 1 0 4z"/>'],["cart","Carrito","compra tienda",`<circle cx="9.5" cy="20" r="1.7" ${b}/><circle cx="18" cy="20" r="1.7" ${b}/><path d="M2 3h3l2.5 12h12L22 6.5H6"/>`],["tag","Etiqueta","precio marcar",`<path d="M11 3H4a1 1 0 0 0-1 1v7l10 10 8-8z"/><circle cx="7.5" cy="7.5" r="1.5" ${b}/>`],["wallet","Cartera","dinero pagos",`<rect x="3" y="6" width="18" height="14" rx="2"/><path d="M3 10h18"/><circle cx="17" cy="15" r="1.4" ${b}/>`],["credit-card","Tarjeta","pago banco",'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M2 10h20M6 15h4"/>'],["coffee","Café","descanso taza",'<path d="M3 8h14v6a5 5 0 0 1-10 0z"/><path d="M17 9h2a2.5 2.5 0 0 1 0 5h-2"/><path d="M3 21h16"/>'],["gamepad","Mando","juego consola",`<rect x="2" y="7" width="20" height="11" rx="4"/><path d="M7 10.5v4M5 12.5h4"/><circle cx="16" cy="11.5" r="1.3" ${b}/><circle cx="18.5" cy="14" r="1.3" ${b}/>`],["dice","Dado","azar juego",`<rect x="3" y="3" width="18" height="18" rx="3"/><circle cx="8" cy="8" r="1.5" ${b}/><circle cx="12" cy="12" r="1.5" ${b}/><circle cx="16" cy="16" r="1.5" ${b}/>`],["trophy","Trofeo","premio ganar",'<path d="M8 3h8v6a4 4 0 0 1-8 0z"/><path d="M8 5H5v1.5A3.5 3.5 0 0 0 8.5 10M16 5h3v1.5A3.5 3.5 0 0 1 15.5 10"/><path d="M12 13v4M8.5 21h7l-1-4h-5z"/>'],["target","Objetivo","diana precision",`<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1.6" ${b}/>`],["timer","Temporizador","cronometro tiempo",'<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4M9.5 2h5"/>'],["hourglass","Reloj de arena","espera tiempo",'<path d="M6 2h12M6 22h12"/><path d="M8 2v4l4 4 4-4V2M8 22v-4l4-4 4 4v4"/>'],["thermometer","Temperatura","calor frio",'<path d="M14 14.2V5a2 2 0 0 0-4 0v9.2a4 4 0 1 0 4 0z"/>'],["wrench","Llave inglesa","herramienta arreglar",'<path d="M20 5.5A5.5 5.5 0 0 1 12.6 13L5 20.6 3.4 19 11 11.4A5.5 5.5 0 0 1 18.5 4L15 7.5l1.5 1.5L20 5.5z"/>'],["hammer","Martillo","herramienta construir",'<path d="M14 3l7 7-3 3-7-7z"/><path d="M11.5 8.5L3 17v4h4l8.5-8.5"/>'],["toolbox","Herramientas","caja utiles",'<rect x="2" y="8" width="20" height="12" rx="2"/><path d="M8 8V5h8v3M2 13h20"/>'],["lightbulb","Idea","bombilla consejo",'<path d="M9 18.5h6M10 21.5h4"/><path d="M12 2.5a6 6 0 0 0-3.5 10.9v2.1h7v-2.1A6 6 0 0 0 12 2.5z"/>'],["magnet","Imán","atraer",'<path d="M6 3H3v9a9 9 0 0 0 18 0V3h-3v9a6 6 0 0 1-12 0z"/><path d="M3 8h3M18 8h3"/>'],["anchor","Ancla","fijar barco",'<circle cx="12" cy="5" r="2.5"/><path d="M12 7.5V21"/><path d="M5 12H3a9 9 0 0 0 18 0h-2"/><path d="M8 11h8"/>'],["plane","Avión","viaje vuelo",'<path d="M10 3.5a2 2 0 0 1 4 0V9l8 4.5v2.5l-8-2.5v4l2.5 2v2L12 20l-4.5 1.5v-2l2.5-2v-4L2 16v-2.5L10 9z"/>'],["car","Coche","vehiculo transporte",`<path d="M3 16v-3.5L5 7h14l2 5.5V16z"/><path d="M4.5 16v3H7v-3M17 16v3h2.5v-3"/><circle cx="7.5" cy="12.5" r="1.3" ${b}/><circle cx="16.5" cy="12.5" r="1.3" ${b}/>`],["book","Libro","leer manual",'<path d="M4 4.5A2.5 2.5 0 0 1 6.5 2H20v16H6.5A2.5 2.5 0 0 0 4 20.5z"/><path d="M4 20.5A2.5 2.5 0 0 1 6.5 18H20v4H6.5A2.5 2.5 0 0 1 4 20.5z"/>'],["bookmark","Marcador","guardar favorito",'<path d="M6 3h12v18l-6-4.5L6 21z"/>'],["graduation","Formación","estudios birrete",'<path d="M2 8l10-4 10 4-10 4z"/><path d="M6 10.5V16c0 1.7 2.7 3 6 3s6-1.3 6-3v-5.5"/>'],["thumbs-up","Me gusta","aprobar bien",'<path d="M7 21V10l5-8a2.5 2.5 0 0 1 2.4 3.2L13.5 9H20a2 2 0 0 1 2 2.4l-1.6 7.2A2 2 0 0 1 18.4 21z"/><rect x="2" y="10" width="5" height="11" rx="1"/>'],["smiley","Sonrisa","cara emoji feliz",`<circle cx="12" cy="12" r="9"/><circle cx="9" cy="10" r="1.3" ${b}/><circle cx="15" cy="10" r="1.3" ${b}/><path d="M7.8 14.3a5 5 0 0 0 8.4 0"/>`],["skull","Calavera","peligro muerte",`<path d="M5 11.5a7 7 0 1 1 14 0V14l-2 1.5V19h-3v-2h-4v2H7v-3.5L5 14z"/><circle cx="9.3" cy="11.5" r="1.8" ${b}/><circle cx="14.7" cy="11.5" r="1.8" ${b}/>`],["crown","Corona","rey premium",'<path d="M3 8l4.5 4L12 4.5 16.5 12 21 8v11H3z"/>']]},ur=Object.entries(Yd).flatMap(([e,t])=>t.map(([a,n,r,i])=>({id:a,name:n,cat:e,keywords:r,body:i})));function Qr(e){return String(e).toLowerCase().normalize("NFD").replace(/[̀-ͯ]/g,"")}function Qd(e="",t="all"){const a=Qr(e).split(/\s+/).filter(Boolean);return ur.filter(n=>{if(t!=="all"&&n.cat!==t)return!1;if(!a.length)return!0;const r=Qr(`${n.id} ${n.name} ${n.keywords}`);return a.every(i=>r.includes(i))})}function qo(e,{size:t=24,color:a=null,strokeWidth:n=2}={}){const r=a?` style="color:${a}"`:"";return`<svg xmlns="http://www.w3.org/2000/svg" width="${t}" height="${t}" viewBox="0 0 24 24"${r}
    fill="none" stroke="currentColor" stroke-width="${n}"
    stroke-linecap="round" stroke-linejoin="round">${e.body}</svg>`}function Zd(e){return ur.find(t=>t.id===e)||null}const Jd=3,eu=28,_o=9,zo=[{css:"Segoe UI",label:"Segoe UI"},{css:"Segoe UI Black",label:"Segoe UI Black"},{css:"Arial",label:"Arial"},{css:"Arial Black",label:"Arial Black"},{css:"Impact",label:"Impact (estrecha)"},{css:"Tahoma",label:"Tahoma"},{css:"Verdana",label:"Verdana"},{css:"Consolas",label:"Consolas (mono)"},{css:"Courier New",label:"Courier New (mono)"},{css:"Georgia",label:"Georgia"},{css:"Times New Roman",label:"Times New Roman"}],tu=[{id:"pencil",icon:"pencil",label:"Lápiz"},{id:"eraser",icon:"eraser",label:"Borrador"},{id:"fill",icon:"fill",label:"Relleno"},{id:"select",icon:"select",label:"Selección (mover/redimensionar)"}],l={profile:0,kind:"key",page:0,key:0,layer:"normal",tool:"pencil",zoom:_o,buffer:Gt(),drawing:!1,drawValue:1,lastPos:null,undoStack:[],dirty:!1,source:null,layerXf:null,dragging:null,selecting:null,outline:null,textDraft:"",textFont:"Segoe UI",textSize:16,textBold:!0,libQuery:"",libCat:"all"};function Bo(){Le();const e=document.getElementById("view-oled");e.addEventListener("click",ru),e.addEventListener("change",iu),e.addEventListener("input",ou),window.addEventListener("keydown",su),W("connected",()=>{$n()});let t=0;it(()=>{s.profiles.length!==t&&(t=s.profiles.length,l.profile>=s.profiles.length&&(l.profile=0),$n(),Le())})}function jo({profile:e,key:t,layer:a,kind:n}={}){Number.isInteger(e)&&e<s.profiles.length&&(l.profile=e),(a==="normal"||a==="super")&&(l.layer=a),l.kind=n==="profile"?"profile":"key",_a(!1),Number.isInteger(t)&&pr(t)&&(l.key=t),l.page=l.kind==="profile"?0:s.profiles[l.profile]?.pageIdx||0;const r=ye(l.profile,ie(),l.page);l.buffer=r?Uint8Array.from(r):Gt(),l.undoStack=[],l.dirty=!1,Le(),$n()}function Do(){_a(!1),rn("view-profiles")}function au(){l.dirty&&!confirm(`Se perderá lo que hayas dibujado en este icono.

¿Salir sin guardar?`)||Do()}function pr(e){return ze[e]}function nu(e,t=l.layer){const a=pr(e);return a?a-1+(t==="super"?10:0):-1}function ie(){return l.kind==="profile"?wa:nu(l.key)}function $n(){De(l.profile)}function ru(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;a==="tool"?(l.tool=t.dataset.tool,Eu()):a==="zoom-in"?sa(l.zoom+xn(l.zoom)):a==="zoom-out"?sa(l.zoom-xn(l.zoom-1)):a==="zoom-fit"?sa(lu()):a==="exit"?au():a==="clear"?(Ye(),ki(l.buffer),q()):a==="invert"?(Ye(),Bs(l.buffer),q()):a==="frame"?(Ye(),Ei(l.buffer),q()):a==="undo"?vu():a==="make-text"?yu():a==="import"?document.getElementById("oled-file").click():a==="apply"?Xo():a==="cancel"?_a(!0):a==="center"?Go():a==="fit-layer"?fu():a==="nudge"?pu(Number(t.dataset.dx),Number(t.dataset.dy)):a==="size-step"?hr(fr()+Number(t.dataset.d)):a==="xf-invert"?(l.layerXf.invert=t.dataset.mode,t.parentElement.querySelectorAll('[data-act="xf-invert"]').forEach(n=>n.classList.toggle("on",n.dataset.mode===t.dataset.mode)),q()):a==="lib-cat"?mu(t.dataset.cat):a==="lib-pick"?gu(t.dataset.id):a==="upload"?xu():a==="reset-slot"?wu():a==="load-current"&&Mu()}function iu(e){e.target.id==="oled-file"&&e.target.files?.[0]?(hu(e.target.files[0]),e.target.value=""):e.target.dataset.act==="text-font"?(l.textFont=e.target.value,l.source?.kind==="text"&&wn()):e.target.dataset.act==="layer-mode"&&(l.layerXf.mode=e.target.value,q())}function ou(e){const t=e.target.dataset.act;if(t)if(t==="lib-search")l.libQuery=e.target.value,Ko();else if(t==="text-draft")l.textDraft=e.target.value;else if(t==="text-size")l.textSize=Number(e.target.value),l.source?.kind==="text"&&wn();else if(t==="text-bold")l.textBold=e.target.checked,l.source?.kind==="text"&&wn();else if(l.layerXf)t==="xf-size"?hr(Number(e.target.value),!1):t==="xf-threshold"?(l.layerXf.threshold=Number(e.target.value),q(),Ie()):t==="xf-blur"?(l.layerXf.blur=Number(e.target.value),q(),Ie()):t==="xf-dither"&&(l.layerXf.dither=e.target.checked,q());else return}function su(e){if(!l.layerXf)return;const t=["INPUT","TEXTAREA","SELECT"].includes(e.target.tagName);if(e.key==="Escape"){_a(!0);return}if(e.key==="Enter"&&!t){e.preventDefault(),Xo();return}if(t)return;const a=e.shiftKey?5:1,n={ArrowLeft:[-a,0],ArrowRight:[a,0],ArrowUp:[0,-a],ArrowDown:[0,a]}[e.key];n&&(e.preventDefault(),l.layerXf.x+=n[0],l.layerXf.y+=n[1],q(),Ie())}function xn(e){return e<8?1:e<16?2:4}function lu(){const e=document.getElementById("oled-canvas-wrap");if(!e)return _o;const t=getComputedStyle(e),a=e.clientWidth-parseFloat(t.paddingLeft)-parseFloat(t.paddingRight),n=e.clientHeight-parseFloat(t.paddingTop)-parseFloat(t.paddingBottom);return Ho(Math.min(a/w,n/S))}function Ho(e){return Math.max(Jd,Math.min(eu,Math.round(e)))}function sa(e,t=null){const a=Ho(e);if(a===l.zoom)return;const n=document.getElementById("oled-canvas"),r=document.getElementById("oled-canvas-wrap");let i=null,o=null;if(t&&n){const c=n.getBoundingClientRect();i=(t.clientX-c.left)/l.zoom,o=(t.clientY-c.top)/l.zoom}if(l.zoom=a,q(),cu(),i!==null&&r&&n){const c=n.getBoundingClientRect();r.scrollLeft+=c.left+i*a-t.clientX,r.scrollTop+=c.top+o*a-t.clientY}}function cu(){const e=document.getElementById("oled-zoom-level");e&&(e.textContent=`${l.zoom}×`)}const Zr=4;function du(){const e=document.getElementById("oled-canvas"),t=document.getElementById("oled-canvas-wrap");if(!e)return;const a=r=>{const i=e.getBoundingClientRect();return{x:(r.clientX-i.left)/i.width*w,y:(r.clientY-i.top)/i.height*S}};t?.addEventListener("wheel",r=>{if(!r.ctrlKey)return;r.preventDefault();const i=r.deltaY<0?1:-1;sa(l.zoom+i*xn(i>0?l.zoom:l.zoom-1),r)},{passive:!1}),e.addEventListener("pointerdown",r=>{r.preventDefault(),e.setPointerCapture(r.pointerId);const i=a(r);if(l.layerXf){const u=Ma(l.source,l.layerXf),p=l.outline||u,h=Math.abs(i.x-(p.x+p.width))<=Zr,v=Math.abs(i.y-(p.y+p.height))<=Zr;l.dragging=h&&v?{kind:"resize",originX:u.x,originY:u.y,startH:u.height,startW:u.width}:{kind:"move",grabX:i.x-u.x,grabY:i.y-u.y};return}if(l.tool==="select"){l.selecting={x0:i.x,y0:i.y,x1:i.x,y1:i.y};return}Ye();const o=Math.floor(i.x),c=Math.floor(i.y);if(l.tool==="fill"){js(l.buffer,o,c,r.button===2?0:1),q();return}l.drawing=!0,l.drawValue=l.tool==="eraser"||r.button===2?0:1,l.lastPos={x:o,y:c},ne(l.buffer,o,c,l.drawValue),q()}),e.addEventListener("pointermove",r=>{const i=a(r);if(l.dragging){if(l.dragging.kind==="move")l.layerXf.x=i.x-l.dragging.grabX,l.layerXf.y=i.y-l.dragging.grabY,q(),Ie();else{const{originX:u,originY:p,startW:h,startH:v}=l.dragging,y=Math.max((i.x-u)/Math.max(h,.001),(i.y-p)/Math.max(v,.001));hr(v*Math.max(.05,y))}return}if(l.selecting){l.selecting.x1=i.x,l.selecting.y1=i.y,q();return}if(!l.drawing)return;const o=Math.floor(i.x),c=Math.floor(i.y);l.lastPos&&Ds(l.buffer,l.lastPos.x,l.lastPos.y,o,c,l.drawValue),l.lastPos={x:o,y:c},q()});const n=()=>{l.drawing=!1,l.lastPos=null,l.dragging=null,l.selecting&&bu()};e.addEventListener("pointerup",n),e.addEventListener("pointercancel",()=>{l.drawing=!1,l.lastPos=null,l.dragging=null,l.selecting=null}),e.addEventListener("contextmenu",r=>r.preventDefault()),e.style.cursor=l.layerXf?"move":"crosshair"}function uu(e){const t=Dn(e),a=ot(e);return{x:(w-a.width*t)/2-a.x*t,y:(S-a.height*t)/2-a.y*t,scale:t,threshold:128,blur:e.kind==="image"&&!e.crisp?1:0,dither:!1,invert:"none",mode:e.kind==="text"?"merge":"replace"}}const Vo=2,Fo=S*3;function Wo(){return Math.max(1,ot(l.source).height)}function fr(){return l.layerXf.scale*Wo()}function hr(e,t=!0){if(!l.layerXf)return;const a=Math.max(Vo,Math.min(Fo,e)),n=Ma(l.source,l.layerXf),r=ot(l.source);l.layerXf.scale=a/Wo(),l.layerXf.x=n.x-r.x*l.layerXf.scale,l.layerXf.y=n.y-r.y*l.layerXf.scale,t&&Uo(),q(),Ie()}function Uo(){const e=document.getElementById("xf-size");e&&(e.value=Math.round(fr()))}function pu(e,t){l.layerXf&&(l.layerXf.x+=e,l.layerXf.y+=t,q(),Ie())}function qa(e=null){return Ys(e||Kt(l.source,l.layerXf))||Ma(l.source,l.layerXf)}function mr(){const e=ot(l.source),t=l.layerXf.scale;l.layerXf.x=Math.round((w-e.width*t)/2)-e.x*t,l.layerXf.y=Math.round((S-e.height*t)/2)-e.y*t;const a=qa();l.layerXf.x+=Math.round((w-a.width)/2-a.x),l.layerXf.y+=Math.round((S-a.height)/2-a.y)}function Go(){mr(),q(),Ie()}const Jr=1;function fu(){l.layerXf.scale=Dn(l.source);for(let e=0;e<6;e++){mr();const t=qa(),a=Math.min((w-2*Jr)/t.width,(S-2*Jr)/t.height);if(Math.abs(a-1)<.02)break;l.layerXf.scale*=Math.max(.5,Math.min(2,a))}Uo(),Go()}function gr(e){l.source=e,l.layerXf=uu(e),mr(),Le()}async function hu(e){try{gr(await Fs(e))}catch(t){g(`No se pudo leer la imagen: ${t.message}`,"error")}}function mu(e){l.libCat=e,document.querySelectorAll('[data-act="lib-cat"]').forEach(t=>t.classList.toggle("on",t.dataset.cat===e)),Ko()}function Ko(){const e=document.getElementById("oled-lib-grid");e&&(e.outerHTML=Qo())}async function gu(e){const t=Zd(e);if(t)try{gr(await Pi(qo(t,{size:256,color:"#fff"})))}catch(a){g(`No se pudo cargar el icono: ${a.message}`,"error")}}function yu(){if(!l.textDraft.trim()){g("Escribe algo primero","error");return}gr(_n(l.textDraft,{fontSize:l.textSize,bold:l.textBold,font:l.textFont}))}function wn(){const e={...l.layerXf};l.source=_n(l.textDraft,{fontSize:l.textSize,bold:l.textBold,font:l.textFont}),l.layerXf=e,q(),Ie()}function bu(){const{x0:e,y0:t,x1:a,y1:n}=l.selecting;l.selecting=null;const r=Math.max(0,Math.floor(Math.min(e,a))),i=Math.max(0,Math.floor(Math.min(t,n))),o=Math.min(w,Math.ceil(Math.max(e,a))),c=Math.min(S,Math.ceil(Math.max(t,n))),u=o-r,p=c-i;if(u<1||p<1){q();return}let h=!1;for(let m=i;m<c&&!h;m++)for(let k=r;k<o;k++)if(mt(l.buffer,k,m)){h=!0;break}if(!h){q();return}Ye();const v=document.createElement("canvas");v.width=u,v.height=p;const y=v.getContext("2d");y.fillStyle="#fff";for(let m=0;m<p;m++)for(let k=0;k<u;k++)mt(l.buffer,r+k,i+m)&&(y.fillRect(k,m,1,1),ne(l.buffer,r+k,i+m,0));l.source={kind:"image",bitmap:v,naturalWidth:u,naturalHeight:p,cut:!0},l.layerXf={x:r,y:i,scale:1,threshold:128,blur:0,dither:!1,invert:"none",mode:"merge"},Le()}function Xo(){if(!l.layerXf)return;Ye();const e=Kt(l.source,l.layerXf);l.buffer=Hn(l.buffer,e,l.layerXf.mode),l.source=null,l.layerXf=null,Le()}function _a(e=!0){l.selecting=null,l.layerXf&&(l.source?.cut&&(l.buffer=l.undoStack.pop()||l.buffer),l.source=null,l.layerXf=null,e&&Le())}function Ye(){l.undoStack.push(l.buffer.slice()),l.undoStack.length>30&&l.undoStack.shift(),l.dirty=!0}function vu(){const e=l.undoStack.pop();e&&(l.buffer=e,q())}function $u(){const{x0:e,y0:t,x1:a,y1:n}=l.selecting;return{x:Math.min(e,a),y:Math.min(t,n),width:Math.abs(a-e),height:Math.abs(n-t)}}function q(){let e=l.buffer;if(l.layerXf){const r=Kt(l.source,l.layerXf);e=Hn(l.buffer,r,l.layerXf.mode),l.outline=qa(r)}else l.outline=l.selecting?$u():null;const t=l.outline,a=document.getElementById("oled-canvas");a&&an(e,a,{zoom:l.zoom,outline:t});const n=document.getElementById("oled-preview");n&&an(e,n,{zoom:2,grid:!1})}function Ie(){if(!l.layerXf)return;const e=(a,n)=>{const r=document.getElementById(a);r&&(r.textContent=n)},t=l.outline||qa();e("xf-size-val",`${Math.round(t.width)} × ${Math.round(t.height)} px`),e("xf-threshold-val",l.layerXf.threshold),e("xf-blur-val",l.layerXf.blur),e("xf-pos-val",`x ${Math.round(t.x)} · y ${Math.round(t.y)}`)}async function xu(){if(!Y())return;if(ie()<0){g("Esa tecla no tiene pantalla","error");return}const e=document.getElementById("btn-oled-upload");e&&(e.disabled=!0);try{l.kind!=="profile"&&await Ze(l.profile,l.page),await ve(l.profile,ie(),l.buffer),Be(l.profile,ie(),Uint8Array.from(l.buffer),l.page);const t=s.profiles[l.profile];t&&(l.kind==="profile"?t.pages[0].oledMask|=1<<ie():t.oledMask|=1<<ie()),B(),l.dirty=!1,g(l.kind==="profile"?"Icono enviado al perfil":`Icono enviado a la tecla ${l.key+1}`),Do()}catch(t){g(`Error al enviar: ${t.message}`,"error"),e&&(e.disabled=!1)}}async function wu(){if(Y())try{l.kind!=="profile"&&await Ze(l.profile,l.page),await Wt(l.profile,ie()),Be(l.profile,ie(),null,l.page);const e=s.profiles[l.profile];e&&(l.kind==="profile"?e.pages[0].oledMask&=~(1<<ie()):e.oledMask&=~(1<<ie())),ki(l.buffer),B(),l.dirty=!1,Le(),g(l.kind==="profile"?"Icono del perfil eliminado":"Icono eliminado; vuelve la etiqueta de texto")}catch(e){g(`Error: ${e.message}`,"error")}}async function Mu(){if(!Y())return;l.kind!=="profile"&&await Ze(l.profile,l.page);const e=await Tn(l.profile,ie());if(!e){g(l.kind==="profile"?"Este perfil no tiene icono guardado":"Esa tecla no tiene icono guardado","info");return}Be(l.profile,ie(),e,l.page),Ye(),l.buffer=Uint8Array.from(e),q(),g("Icono cargado desde el teclado")}function Le(){const e=document.getElementById("view-oled");e&&(e.innerHTML=`
    ${ku()}

    <div class="oled-grid">
      <div class="glass-panel oled-canvas-card">
        <div class="oled-toolbar" id="oled-toolbar">${Yo()}</div>
        <div class="oled-canvas-wrap" id="oled-canvas-wrap">
          <canvas id="oled-canvas"></canvas>
        </div>
        <p class="oled-hint">${Pu()}</p>
      </div>

      <div class="oled-side">
        ${l.layerXf?Iu():Su()}
        ${Lu()}
      </div>
    </div>`,du(),q(),Ie())}function ku(){const e=s.profiles[l.profile];if(l.kind==="profile")return`
      <header class="oled-actionbar">
        <div class="oled-actionbar-info">
          <h1>Icono del perfil</h1>
          <p>${Dt(e?.name||`Perfil ${l.profile+1}`)}</p>
        </div>
        <div class="oled-actionbar-btns">
          <button class="secondary-btn" data-act="exit">${f("close",16)} Salir sin guardar</button>
          <button class="primary-btn" id="btn-oled-upload" data-act="upload">
            ${f("check",16)} Enviar al perfil
          </button>
        </div>
      </header>`;const t=rt()&&e&&D(e)>1?` · página ${l.page+1} de ${D(e)}`:"",a=`${Dt(e?.name||`Perfil ${l.profile+1}`)} · tecla ${l.key+1} · pantalla ${pr(l.key)||"—"} · capa ${l.layer==="super"?"SUPER":"NORMAL"}${t}`;return`
    <header class="oled-actionbar">
      <div class="oled-actionbar-info">
        <h1>Icono de la tecla ${l.key+1}</h1>
        <p>${a}</p>
      </div>
      <div class="oled-actionbar-btns">
        <button class="secondary-btn" data-act="exit">${f("close",16)} Salir sin guardar</button>
        <button class="primary-btn" id="btn-oled-upload" data-act="upload" ${ie()>=0?"":"disabled"}>
          ${f("check",16)} Enviar a la tecla
        </button>
      </div>
    </header>`}function Yo(){const e=l.layerXf?"disabled":"";return`
    ${tu.map(t=>`
      <button class="tool-btn ${l.tool===t.id?"on":""}" data-act="tool" data-tool="${t.id}" title="${t.label}" ${e}>
        ${f(t.icon,18)}
      </button>`).join("")}
    <span class="tool-sep"></span>
    <button class="tool-btn" data-act="frame"  title="Marco" ${e}>${f("square",18)}</button>
    <button class="tool-btn" data-act="invert" title="Invertir" ${e}>${f("invert",18)}</button>
    <button class="tool-btn" data-act="undo"   title="Deshacer" ${e}>${f("reset",18)}</button>
    <button class="tool-btn danger" data-act="clear" title="Vaciar" ${e}>${f("trash",18)}</button>

    <span class="zoom-group">
      <button class="tool-btn" data-act="zoom-out" title="Alejar">${f("minus",18)}</button>
      <span class="zoom-level" id="oled-zoom-level">${l.zoom}×</span>
      <button class="tool-btn" data-act="zoom-in" title="Acercar">${f("plus",18)}</button>
      <button class="tool-btn" data-act="zoom-fit" title="Ajustar a la ventana">${f("fit",18)}</button>
    </span>`}function Eu(){const e=document.getElementById("oled-toolbar");e&&(e.innerHTML=Yo())}function Pu(){return l.layerXf?`Arrastra para mover la capa, y tira de la esquina inferior derecha para escalarla.
            La cruceta y las <strong>flechas</strong> del teclado mueven 1 px (con Shift, 5).
            <strong>Enter</strong> fija, <strong>Esc</strong> cancela.`:`Clic izquierdo pinta, clic derecho borra. <strong>Ctrl + rueda</strong> hace zoom sobre el punto
          del cursor. Las líneas violetas marcan las páginas de 8 px en las que el SSD1306 direcciona la memoria.`}function Qo(){const e=Qd(l.libQuery,l.libCat);return e.length?`
    <div class="icon-lib-grid" id="oled-lib-grid">
      ${e.map(t=>`
        <button class="icon-lib-item" data-act="lib-pick" data-id="${t.id}" title="${Dt(t.name)}">
          ${qo(t,{size:28})}
        </button>`).join("")}
    </div>`:'<div class="icon-lib-grid empty" id="oled-lib-grid">Ningún icono coincide con la búsqueda.</div>'}function Cu(){const e=ur.length;return`
    <div class="glass-panel oled-card">
      <div class="card-header">${f("square",20)}<h2>Biblioteca de iconos</h2></div>
      <p class="setting-desc mb-8">${e} iconos listos para usar. Pincha en uno y colócalo como quieras.</p>

      <input type="search" class="text-input" data-act="lib-search" placeholder="Buscar: guardar, volumen, carpeta…"
             value="${Dt(l.libQuery)}">

      <div class="chip-row icon-lib-cats">
        <button class="chip ${l.libCat==="all"?"on":""}" data-act="lib-cat" data-cat="all">Todos</button>
        ${Xd.map(t=>`
          <button class="chip ${l.libCat===t.id?"on":""}" data-act="lib-cat" data-cat="${t.id}">${t.label}</button>`).join("")}
      </div>

      ${Qo()}
    </div>`}function Su(){return`
    ${Cu()}
    <div class="glass-panel oled-card">
      <div class="card-header">${f("text",20)}<h2>Generar contenido</h2></div>

      <label class="field">
        <span class="field-label">Texto</span>
        <textarea class="text-input" rows="2" data-act="text-draft"
                  placeholder="Enter = salto de línea">${Dt(l.textDraft)}</textarea>
      </label>

      <label class="field">
        <span class="field-label">Fuente</span>
        <select class="select-input" data-act="text-font">
          ${zo.map(e=>`
            <option value="${e.css}" ${l.textFont===e.css?"selected":""}
                    style="font-family:'${e.css}'">${e.label}</option>`).join("")}
        </select>
      </label>

      <div class="row-inline">
        <label class="field-inline"><span>Tamaño</span>
          <input type="number" class="text-input compact" data-act="text-size"
                 value="${l.textSize}" min="6" max="40">
        </label>
        <label class="check"><input type="checkbox" data-act="text-bold" ${l.textBold?"checked":""}> Negrita</label>
        <button class="secondary-btn" data-act="make-text">${f("text",16)} Colocar texto</button>
      </div>

      <div class="divider"></div>

      <button class="secondary-btn full" data-act="import">${f("upload",16)} Importar imagen (PNG / JPG / SVG)</button>
      <input type="file" id="oled-file" accept="image/*" hidden>
      <p class="setting-desc">Podrás moverla y escalarla dentro del recuadro antes de fijarla.</p>
    </div>`}function Iu(){const e=l.layerXf,t=l.source.kind==="text";return`
    <div class="glass-panel oled-card is-placing">
      <div class="card-header">${f("fit",20)}<h2>Colocando ${t?"texto":"imagen"}</h2></div>

      ${t?`
        <label class="field">
          <span class="field-label">Fuente</span>
          <select class="select-input" data-act="text-font">
            ${zo.map(a=>`
              <option value="${a.css}" ${l.textFont===a.css?"selected":""}
                      style="font-family:'${a.css}'">${a.label}</option>`).join("")}
          </select>
        </label>
        <div class="row-inline">
          <label class="field-inline"><span>Tamaño</span>
            <input type="number" class="text-input compact" data-act="text-size" value="${l.textSize}" min="6" max="40">
          </label>
          <label class="check"><input type="checkbox" data-act="text-bold" ${l.textBold?"checked":""}> Negrita</label>
        </div>
        <div class="divider"></div>`:""}

      <div class="field">
        <span class="field-label">Tamaño <b id="xf-size-val"></b></span>
        <div class="size-row">
          <button class="tool-btn small" data-act="size-step" data-d="-1" title="1 px menos">${f("minus",14)}</button>
          <input type="range" class="premium-slider" id="xf-size" data-act="xf-size"
                 min="${Vo}" max="${Fo}" step="1" value="${Math.round(fr())}">
          <button class="tool-btn small" data-act="size-step" data-d="1" title="1 px más">${f("plus",14)}</button>
        </div>
      </div>

      <div class="field">
        <span class="field-label">Posición <b id="xf-pos-val"></b></span>
        <div class="nudge-row">
          <div class="nudge-pad">
            <button class="tool-btn small" data-act="nudge" data-dx="0"  data-dy="-1" title="Arriba 1 px">↑</button>
            <button class="tool-btn small" data-act="nudge" data-dx="-1" data-dy="0"  title="Izquierda 1 px">←</button>
            <button class="tool-btn small" data-act="center" title="Centrar">${f("fit",14)}</button>
            <button class="tool-btn small" data-act="nudge" data-dx="1"  data-dy="0"  title="Derecha 1 px">→</button>
            <button class="tool-btn small" data-act="nudge" data-dx="0"  data-dy="1"  title="Abajo 1 px">↓</button>
          </div>
          <button class="secondary-btn" data-act="fit-layer">Encajar</button>
        </div>
      </div>

      <label class="field">
        <span class="field-label">Umbral de blanco/negro <b id="xf-threshold-val">${e.threshold}</b></span>
        <input type="range" class="premium-slider" data-act="xf-threshold"
               min="8" max="248" step="1" value="${e.threshold}">
      </label>

      <label class="field">
        <span class="field-label">Suavizado de bordes <b id="xf-blur-val">${e.blur}</b></span>
        <input type="range" class="premium-slider" data-act="xf-blur"
               min="0" max="3" step="1" value="${e.blur}">
      </label>

      <label class="check"><input type="checkbox" data-act="xf-dither" ${e.dither?"checked":""}> Difuminado (solo para fotos)</label>

      <div class="field mt-4">
        <span class="field-label">Invertir</span>
        <div class="chip-row">
          <button class="chip ${e.invert==="none"?"on":""}"   data-act="xf-invert" data-mode="none">No</button>
          <button class="chip ${e.invert==="colors"?"on":""}" data-act="xf-invert" data-mode="colors"
                  title="Voltea el color del contenido y respeta la transparencia">Colores</button>
          <button class="chip ${e.invert==="box"?"on":""}"    data-act="xf-invert" data-mode="box"
                  title="Voltea el rectángulo de la capa: fondo encendido, contenido apagado">Recuadro</button>
        </div>
        <p class="setting-desc">Solo afecta a la capa; el resto del lienzo no se toca.</p>
      </div>

      <label class="field mt-4">
        <span class="field-label">Al fijar</span>
        <select class="select-input" data-act="layer-mode">
          <option value="replace" ${e.mode==="replace"?"selected":""}>Sustituir el lienzo</option>
          <option value="merge"   ${e.mode==="merge"?"selected":""}>Combinar con lo dibujado</option>
        </select>
      </label>

      <div class="layer-actions">
        <button class="primary-btn full" data-act="apply">${f("check",16)} Fijar en el lienzo</button>
        <button class="secondary-btn full" data-act="cancel">${f("close",16)} Cancelar</button>
      </div>
    </div>`}function Lu(){return`
    <div class="glass-panel oled-card">
      <div class="card-header">${f("oled",20)}<h2>Pantalla</h2></div>
      <div class="oled-preview-box">
        <span class="field-label">Tamaño real</span>
        <canvas id="oled-preview" width="${w*2}" height="${S*2}"></canvas>
      </div>
      <div class="row-inline">
        <button class="secondary-btn" data-act="load-current">${f("refresh",16)} Releer del teclado</button>
        <button class="secondary-btn danger" data-act="reset-slot">${f("trash",16)} Quitar icono</button>
      </div>
      <p class="setting-desc">Recuerda pulsar <strong>Guardar en Flash</strong> para que sobreviva a la desconexión.</p>
    </div>`}function Dt(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}const Au=Object.freeze(Object.defineProperty({__proto__:null,init:Bo,openTarget:jo,render:Le},Symbol.toStringTag,{value:"Module"})),$={available:!1,enabled:!1,rules:[],fallback:null,current:null,lastApplied:null,error:null},Tu=[{match:"photoshop",label:"Photoshop"},{match:"altium",label:"Altium Designer"},{match:"premiere",label:"Premiere Pro"},{match:"code",label:"VS Code"},{match:"excel",label:"Excel"},{match:"winword",label:"Word"},{match:"chrome",label:"Chrome"},{match:"blender",label:"Blender"}];async function Zo(){const e=document.getElementById("view-auto");e.addEventListener("click",Ou),e.addEventListener("change",qu),$.available=await window.orby.foreground.available();const t=await window.orby.getConfig();Object.assign($,{enabled:t.autoProfile.enabled,rules:t.autoProfile.rules||[],fallback:t.autoProfile.fallback}),ln($.enabled),Ri(()=>dt()),window.orby.foreground.onChange(n=>{$.current=n,Mn(n),dt()}),window.orby.foreground.onError(n=>{$.error=n,dt()}),W("connected",()=>setTimeout(()=>Mn($.current),1500));let a=0;it(()=>{s.profiles.length!==a?(a=s.profiles.length,Nu(),Se()):dt()}),$.enabled&&($.current=await window.orby.foreground.current()),Se()}function Ru(e,t){return e?t==="title"?(e.title||"").toLowerCase():t==="process"?(e.process||"").toLowerCase():`${e.process||""} ${e.title||""}`.toLowerCase():""}function Jo(e){for(const t of $.rules){const a=(t.match||"").trim().toLowerCase();if(a&&Ru(e,t.field).includes(a))return t}return null}function Nu(){const e=s.profiles.length-1;if(e<0)return;let t=!1;for(const a of $.rules)a.profile>e&&(a.profile=e,t=!0);$.fallback!==null&&$.fallback!==void 0&&$.fallback>e&&($.fallback=e,t=!0),t&&qe()}async function Mn(e){if(!$.enabled||!s.connected)return;Ml(e);const t=Jo(e),a=t?t.profile:$.fallback;try{a!=null&&a!==s.activeProfileIdx&&(await ci(a),s.activeProfileIdx=a,te()),$.lastApplied=t?.id??"fallback",await _i(),dt()}catch{}}function qe(){return window.orby.setConfig({autoProfile:{enabled:$.enabled,rules:$.rules,fallback:$.fallback}})}async function Ou(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;if(a==="toggle")$.enabled=!$.enabled,ln($.enabled),await qe(),$.enabled?await window.orby.foreground.start()?($.current=await window.orby.foreground.current(),Mn($.current)):(g("No se pudo iniciar el detector de aplicaciones","error"),$.enabled=!1,ln(!1),await qe()):(await window.orby.foreground.stop(),await xt()),Se();else if(a==="add-current"){const n=$.current?.process;if(!n){g("Aún no se ha detectado ninguna ventana","error");return}if($.rules.some(r=>r.match===n.toLowerCase())){g("Esa aplicación ya tiene tarjeta","info");return}Ga(n.toLowerCase(),s.activeProfileIdx)}else if(a==="add-suggestion")Ga(t.dataset.match,s.activeProfileIdx);else if(a==="add-blank")Ga("",s.activeProfileIdx);else if(a==="remove")$.rules=$.rules.filter(n=>n.id!==t.dataset.id),qe(),Se();else if(a==="move-up"||a==="move-down"){const n=$.rules.findIndex(i=>i.id===t.dataset.id),r=a==="move-up"?n-1:n+1;if(n<0||r<0||r>=$.rules.length)return;[$.rules[n],$.rules[r]]=[$.rules[r],$.rules[n]],qe(),Se()}}function Ga(e,t){$.rules.push({id:`r${Date.now()}${Math.random().toString(36).slice(2,6)}`,match:e,profile:t??0,field:"any"}),qe(),Se()}function qu(e){const t=e.target,a=t.dataset.id;if(t.dataset.act==="rule-match"||t.dataset.act==="rule-profile"||t.dataset.act==="rule-field"){const n=$.rules.find(r=>r.id===a);if(!n)return;t.dataset.act==="rule-match"&&(n.match=t.value.trim()),t.dataset.act==="rule-profile"&&(n.profile=Number(t.value)),t.dataset.act==="rule-field"&&(n.field=t.value),qe(),Se()}else t.dataset.act==="fallback"&&($.fallback=t.value===""?null:Number(t.value),qe(),Se())}function Ht(){return s.profiles.length?s.profiles.map(e=>e.name):["P1","P2","P3","P4"]}function es(e,t=!1){const a=Ht();return(t?`<option value="" ${e===null?"selected":""}>— no cambiar —</option>`:"")+a.map((r,i)=>`<option value="${i}" ${e===i?"selected":""}>${Z(r)}</option>`).join("")}function dt(){const e=document.getElementById("auto-status");if(!e)return;const t=$.current,a=t?Jo(t):null,n=Ht(),r=yl();e.innerHTML=`
    <div class="auto-now ${a?"matched":""}">
      <span class="field-label">Ventana activa</span>
      <strong>${Z(t?.process||"—")}</strong>
      <em>${Z(t?.title||"Sin detección todavía")}</em>
      <span class="auto-verdict">
        ${a?`Encaja con <code>${Z(a.match)}</code> → <b>${Z(n[a.profile]??"?")}</b>`:$.fallback!==null&&$.fallback!==void 0?`Sin regla → vuelve a <b>${Z(n[$.fallback]??"?")}</b>`:"Sin regla que encaje"}
      </span>
      ${r?`
        <span class="auto-verdict">
          Variación <b>${Z(r.name)}</b> aplicada sobre
          ${Z(n[r.profile]??"?")}
          (${Ot(r)}
           ${Ot(r)===1?"cambio":"cambios"})
        </span>`:""}
    </div>
    ${$.error?`<p class="auto-error">${Z($.error)}</p>`:""}`}function _u(e,t){const a=la(e.profile),n=Ht(),r=e.match||"Sin programa";return`
    <div class="rule-card ${$.lastApplied===e.id?"is-active":""}" style="--accent:${a.accent}">
      <div class="rule-card-head">
        <span class="rule-app-icon">${f(a.icon,18)}</span>
        <span class="rule-app-name" title="${Z(r)}">${Z(r)}</span>
        <span class="rule-priority">#${t+1}</span>
      </div>

      <label class="field">
        <span class="field-label">Programa o texto de la ventana</span>
        <input type="text" class="text-input" placeholder="p. ej. photoshop"
               value="${Z(e.match)}" data-act="rule-match" data-id="${e.id}">
      </label>

      <label class="field">
        <span class="field-label">Usa el perfil</span>
        <select class="select-input" data-act="rule-profile" data-id="${e.id}">
          ${es(e.profile)}
        </select>
      </label>

      <div class="rule-card-foot">
        <select class="select-input compact" data-act="rule-field" data-id="${e.id}">
          <option value="any"     ${e.field==="any"?"selected":""}>Programa o título</option>
          <option value="process" ${e.field==="process"?"selected":""}>Solo programa</option>
          <option value="title"   ${e.field==="title"?"selected":""}>Solo título</option>
        </select>
        <div class="rule-card-btns">
          <button class="tool-btn small" data-act="move-up"   data-id="${e.id}" title="Más prioridad">↑</button>
          <button class="tool-btn small" data-act="move-down" data-id="${e.id}" title="Menos prioridad">↓</button>
          <button class="tool-btn small danger" data-act="remove" data-id="${e.id}" title="Eliminar">${f("trash",14)}</button>
        </div>
      </div>

      ${$.lastApplied===e.id?`<span class="rule-card-live">Aplicando ${Z(n[e.profile]??"?")}</span>`:""}
    </div>`}function Se(){const e=document.getElementById("view-auto");if(!e)return;if(!$.available){e.innerHTML=`
      <div class="empty-panel glass-panel">
        ${f("info",40)}
        <h3>Solo disponible en Windows</h3>
        <p>La detección de la ventana activa usa la API de Windows.</p>
      </div>`;return}const t=$.fallback===null||$.fallback===void 0?null:Ht()[$.fallback]??"?";e.innerHTML=`
    <div class="auto-grid">
      <div class="auto-main-col">
        <div class="glass-panel auto-main">
          <div class="auto-head">
            <div class="card-header">${f("bolt",22)}<h2>Aplicaciones</h2></div>
            <button class="switch ${$.enabled?"on":""}" data-act="toggle" title="Activar o desactivar">
              <span class="switch-knob"></span>
            </button>
          </div>

          <div class="row-inline">
            <button class="primary-btn" data-act="add-current">${f("plug",16)} Añadir la app actual</button>
            <button class="secondary-btn" data-act="add-blank">${f("plus",16)} Tarjeta vacía</button>
          </div>

          <div class="field mt-4">
            <span class="field-label">Añadir rápido</span>
            <div class="chip-row">
              ${Tu.map(a=>`
                <button class="chip" data-act="add-suggestion" data-match="${a.match}">${a.label}</button>`).join("")}
            </div>
          </div>

          <div class="rule-cards">
            ${$.rules.length?$.rules.map(_u).join(""):`<div class="rule-empty">
                   Todavía no hay tarjetas. Añade la aplicación que tengas abierta y elige su perfil.
                 </div>`}
          </div>

          ${$.rules.length>1?`
            <p class="setting-desc mt-4">
              Si dos tarjetas encajan a la vez gana la de más arriba: usa las flechas para
              poner primero las más específicas. La comparación no distingue mayúsculas.
            </p>`:""}
        </div>

        <div class="glass-panel auto-main fallback-card">
          <div class="card-header">${f("profiles",22)}<h2>Perfil por defecto</h2></div>
          <div class="fallback-row">
            <label class="field">
              <span class="field-label">Cuando no se detecta ninguna de las apps añadidas</span>
              <select class="select-input" data-act="fallback">${es($.fallback??null,!0)}</select>
            </label>
            <span class="fallback-pill ${t?"on":""}">
              ${t?Z(t):"No cambiar"}
            </span>
          </div>
        </div>
      </div>

      <div class="glass-panel auto-side">
        <div class="card-header">${f("dashboard",22)}<h2>Estado</h2></div>
        <div id="auto-status"></div>

        <div class="auto-legend">
          <span class="field-label">Perfiles</span>
          ${Ht().map((a,n)=>`
            <div class="legend-row ${s.activeProfileIdx===n?"on":""}">
              <span class="legend-dot" style="background:${la(n).accent}"></span>
              <span>${Z(a)}</span>
              ${s.activeProfileIdx===n?"<em>activo</em>":""}
            </div>`).join("")}
        </div>

        <p class="setting-desc mt-4">
          La detección corre en un proceso de PowerShell que consulta la ventana
          activa cada 400 ms. Solo se manda un comando al teclado cuando el
          perfil realmente tiene que cambiar.
        </p>
        <p class="setting-desc">
          Estas tarjetas se guardan en tu PC, no en el teclado: no necesitan
          «Guardar en Flash».
        </p>

        <div class="divider"></div>
        <p class="setting-desc">
          <strong>¿Y si solo cambia un atajo?</strong> Aquí eliges <em>qué perfil</em>
          se activa con cada app. Si lo que quieres es que un perfil concreto tenga
          un par de teclas distintas en una app —«seleccionar todo» con Ctrl+E en vez
          de Ctrl+A, por ejemplo— no hace falta duplicar el perfil: crea una
          <strong>variación</strong> desde <em>Perfiles y macros</em>. Guarda solo las
          diferencias y se aplica sola encima del perfil cuando esa app está delante.
        </p>
      </div>
    </div>`,dt()}function Z(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}const zu=Object.freeze(Object.defineProperty({__proto__:null,init:Zo,render:Se},Symbol.toStringTag,{value:"Module"}));let Ka=0,ei=null;function ti(e){const t=document.getElementById("backup-status"),a=document.getElementById("btn-backup-save"),n=document.getElementById("btn-backup-load");t&&(t.textContent=e||""),a&&(a.disabled=!!e),n&&(n.disabled=!!e)}function ts(){const e=(n,r)=>V(n)?!0:(document.querySelector(r)?.classList.add("hidden"),!1),t=e("autostart","#settings-autostart"),a=e("plugins","#settings-plugins");e("appUpdate","#settings-app"),e("firmwareUpdate","#settings-firmware"),document.getElementById("btn-backup-save").addEventListener("click",()=>Rl(ti)),document.getElementById("btn-backup-load").addEventListener("click",()=>Nl(ti)),document.querySelectorAll("#timeout-selector .opt-btn").forEach(n=>{n.addEventListener("click",async()=>{const r=Number(n.dataset.val);if(Y())try{await ps(r),s.timeout=r,B(),xa()}catch{g("El teclado no confirmó el tiempo de reposo","error")}})}),t&&Du(),a&&Hu(),document.getElementById("btn-reconnect").addEventListener("click",()=>{window.orby.reconnect(),g("Buscando el dispositivo…","info")}),document.getElementById("btn-resync").addEventListener("click",async()=>{try{await fe(),g("Configuración releída del teclado")}catch{g("No se pudo leer la configuración","error")}}),document.getElementById("btn-factory-reset").addEventListener("click",async()=>{if(Y()&&confirm(`Se restaurarán los perfiles, iconos y la calibración de fábrica en la memoria del teclado.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`))try{await hs(),ka(),await fe(),B(),g("Valores de fábrica restaurados (sin guardar todavía)","info")}catch{g("No se pudo restaurar la configuración","error")}}),Gu(),Bu(),ju()}function Bu(){const e=document.getElementById("btn-check-update"),t=document.getElementById("btn-install-update");!e||!t||(document.getElementById("app-update-desc").textContent="Se comprueba al arrancar y cada seis horas. Las versiones nuevas se descargan e instalan solas: la app se cierra un momento y vuelve a abrirse sola. Si hay cambios sin guardar en la Flash del teclado, espera a que se guarden.",e.addEventListener("click",async()=>{if(re.status==="dev"){g("En modo desarrollo no hay actualizaciones que buscar","info");return}await Ul(),g("Buscando actualizaciones…","info")}),t.addEventListener("click",()=>Fi()),Vi(kn),kn())}function kn(){if(!V("appUpdate"))return;const e=document.getElementById("app-version"),t=document.getElementById("app-update-status"),a=document.getElementById("btn-install-update");if(!e||!t||!a)return;e.textContent=re.version||"—",t.textContent=Gl(),t.style.color=re.status==="error"?"var(--danger)":"";const n=re.status;a.classList.toggle("hidden",n!=="downloaded"&&n!=="available")}function ju(){const e=document.getElementById("settings-firmware");if(e){if(!La()){e.classList.add("hidden");return}document.getElementById("btn-fw-check").addEventListener("click",async()=>{const t=await Qi();if(!t){g("La actualización de firmware no está disponible en esta sesión","error",6e3);return}if(t.status==="error"){g(t.error,"error",6e3);return}t.status==="idle"&&!t.available&&g(t.latest?`El teclado ya tiene el último firmware (${t.latest.version})`:"No hay firmware publicado que esta versión de la app sepa instalar","info")}),document.getElementById("btn-fw-update").addEventListener("click",async()=>{if(!Y())return;if(s.dirty){g("Guarda los cambios en Flash antes de actualizar el firmware","error",6e3);return}const a=!lt(s.deviceInfo,"bootsel")?`Este firmware no sabe reiniciarse solo: cuando te lo pida, desenchufa el teclado y vuelve a enchufarlo con BOOTSEL pulsado.

`:"";confirm(`${a}El teclado dejará de funcionar durante la copia (unos segundos). No lo desconectes.

¿Actualizar a la ${G.latest?.version}?`)&&(await ac(),G.status==="done"?g("Firmware actualizado"):G.status==="error"&&g(G.error,"error",9e3))}),document.getElementById("btn-fw-cancel").addEventListener("click",()=>{nc(),g("Actualización cancelada","info")}),ec(En),tc().then(En)}}function En(){if(!V("firmwareUpdate"))return;const e=document.getElementById("fw-current");if(!e)return;const t=G,a=document.getElementById("fw-latest"),n=document.getElementById("fw-status"),r=document.getElementById("btn-fw-update"),i=document.getElementById("btn-fw-check"),o=document.getElementById("btn-fw-cancel");e.textContent=s.connected?s.deviceInfo?.fw||"?":"sin teclado",a.textContent=t.latest?.version||"—",n.textContent=rc(),n.style.color=t.status==="error"?"var(--danger)":"",i.disabled=Lr(),o.classList.toggle("hidden",!["downloading","bootsel"].includes(t.status)),r.classList.toggle("hidden",!t.latest||!s.connected||Lr()),document.getElementById("fw-update-label").textContent=t.available?`Actualizar a ${t.latest.version}`:"Reinstalar firmware"}async function Du(){const e=document.getElementById("btn-autostart");e&&(e.classList.toggle("on",await window.orby.autostart.get()),e.addEventListener("click",async()=>{const t=await window.orby.autostart.set(!e.classList.contains("on"));e.classList.toggle("on",t),g(t?"OrbyGUI arrancará con Windows":"Autoarranque desactivado")}))}function Hu(){const e=document.getElementById("btn-plugin-install"),t=document.getElementById("btn-plugin-folder"),a=document.getElementById("plugin-list"),n=document.getElementById("plugin-settings-cards");!e||!a||!n||(e.addEventListener("click",async()=>{e.disabled=!0;const r=await window.orby.plugins.install();if(e.disabled=!1,!r.canceled){if(!r.ok){g(`No se pudo instalar: ${r.error}`,"error",6e3);return}await ia(),g(`Complemento «${r.plugin.name}» instalado`)}}),t?.addEventListener("click",()=>window.orby.plugins.openFolder()),a.addEventListener("click",async r=>{const i=r.target.closest("[data-act]");if(!i)return;const o=i.dataset.plugin;if(i.dataset.act==="plugin-enable"){const c=Je(o);await window.orby.plugins.setEnabled(o,!c?.enabled),await ia()}else if(i.dataset.act==="plugin-remove"){const c=Je(o);if(!confirm(`Se desinstalará «${c?.name||o}».

Las teclas y mandos que lo usaran dejarán de hacer nada, pero conservarán su ajuste: si vuelves a instalarlo, volverán a funcionar.

¿Continuar?`))return;const u=await window.orby.plugins.uninstall(o);if(!u.ok){g(`No se pudo desinstalar: ${u.error}`,"error",5e3);return}await ia(),g("Complemento desinstalado")}}),n.addEventListener("change",r=>{const i=r.target.closest("[data-plugin][data-key]");if(!i)return;const o=i.type==="number"?Number(i.value):i.value;window.orby.plugins.setSettings(i.dataset.plugin,{[i.dataset.key]:o})}),n.addEventListener("click",async r=>{const i=r.target.closest("[data-act]");if(i)if(i.dataset.act==="plugin-toggle-field"){const o=!i.classList.contains("on");i.classList.toggle("on",o),window.orby.plugins.setSettings(i.dataset.plugin,{[i.dataset.key]:o})}else i.dataset.act==="plugin-test"&&await Vu(i.dataset.plugin,i)}),Jn(ai),ai())}async function Vu(e,t){const a=document.getElementById(`plugin-status-${e}`),n={};document.querySelectorAll(`#plugin-settings-cards [data-plugin="${e}"][data-key]`).forEach(i=>{n[i.dataset.key]=i.classList?.contains("switch")?i.classList.contains("on"):i.type==="number"?Number(i.value):i.value}),window.orby.plugins.setSettings(e,n),a&&(a.textContent="Probando…"),t.disabled=!0;const r=await window.orby.plugins.test(e,n);t.disabled=!1,a&&(a.textContent=r.ok?r.detail:`Sin respuesta (${r.error})`),g(r.ok?"El complemento responde":"El complemento no responde",r.ok?"success":"error")}function ai(){V("plugins")&&(Fu(),Wu())}function Fu(){const e=document.getElementById("plugin-list");if(!e)return;const t=Xl();if(!t.length){e.innerHTML='<p class="plugin-empty">Todavía no hay ninguno instalado.</p>';return}e.innerHTML=t.map(a=>`
    <div class="plugin-row ${a.error?"has-error":""}">
      <span class="plugin-row-icon">${f(a.icon,18)}</span>
      <div class="plugin-row-main">
        <span class="plugin-row-name">${ae(a.name)} <b>${ae(a.version)}</b></span>
        <span class="plugin-row-desc">${ae(a.error?`No se pudo cargar: ${a.error}`:a.description)}</span>
      </div>
      <div class="plugin-row-actions">
        ${a.error?"":`
          <button class="switch ${a.enabled?"on":""}" data-act="plugin-enable" data-plugin="${a.id}"
                  title="${a.enabled?"Desactivar":"Activar"}">
            <span class="switch-knob"></span>
          </button>`}
        <button class="tool-btn small danger" data-act="plugin-remove" data-plugin="${a.id}"
                title="Desinstalar">${f("trash",14)}</button>
      </div>
    </div>`).join("")}function Wu(){const e=document.getElementById("plugin-settings-cards");if(!e)return;const t=er().filter(a=>a.settings?.fields?.length||a.settings?.hasTest);e.innerHTML=t.map(a=>`
    <div class="settings-card glass-panel">
      <div class="card-header">
        <span>${f(a.icon,22)}</span>
        <h2>${ae(a.name)}</h2>
      </div>
      <div class="setting-body">
        ${(a.settings.fields||[]).map(n=>Uu(a,n)).join("")}
        ${a.settings.hasTest?`
          <div class="row-inline">
            <button class="secondary-btn" data-act="plugin-test" data-plugin="${a.id}">Probar</button>
            <span class="setting-desc" id="plugin-status-${a.id}"></span>
          </div>`:""}
        ${a.settings.description?`<p class="setting-desc">${ae(a.settings.description)}</p>`:""}
      </div>
    </div>`).join("")}function Uu(e,t){const a=e.values?.[t.key],n=`data-plugin="${e.id}" data-key="${ae(t.key)}"`;if(t.type==="toggle")return`
      <div class="row-inline">
        <button class="switch ${a?"on":""}" data-act="plugin-toggle-field" ${n}>
          <span class="switch-knob"></span>
        </button>
        <span class="switch-label">${ae(t.label)}</span>
      </div>
      ${t.hint?`<p class="setting-desc">${ae(t.hint)}</p>`:""}`;const r=t.type==="select"?`<select class="select-input" ${n}>
         ${t.options.map(i=>`
           <option value="${ae(i.value)}" ${String(a)===i.value?"selected":""}>${ae(i.label)}</option>`).join("")}
       </select>`:`<input class="text-input" type="${t.type==="number"?"number":t.type}" ${n}
              value="${ae(a??"")}" placeholder="${ae(t.placeholder)}" spellcheck="false">`;return`
    <label class="field">
      <span class="field-label">${ae(t.label)}</span>
      ${r}
    </label>
    ${t.hint?`<p class="setting-desc">${ae(t.hint)}</p>`:""}`}function ae(e){return String(e??"").replace(/[&<>"']/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[t])}function Gu(){const e=document.getElementById("settings-wheel-calib-body");e&&(e.addEventListener("click",t=>{const a=t.target.closest("[data-act]");if(!a)return;const n=a.dataset.act;n==="dial-marker"?Xa({marker:a.dataset.marker}):n==="dial-invert"?Xa({invert:!$e.invert}):n==="dial-nudge"?Xa({offsetDeg:Kn($e.offsetDeg+Number(a.dataset.d))}):n==="dial-align"&&(fl(),$a(),Ra())}),e.addEventListener("input",t=>{if(t.target.dataset.act!=="dial-offset")return;const a=Number(t.target.value);Un({offsetDeg:a});const n=document.getElementById("dial-offset-val");n&&(n.textContent=`${a}°`)}),Ii(as),W("telemetry",t=>{t.startsWith("WHEEL:")&&Ku(parseInt(t.slice(6),10))}),$a())}function as(e){const t=document.getElementById("settings-wheel-needle");t&&(t.style.transform=`rotate(${e}deg)`)}function Ku(e){if(!Number.isFinite(e))return;Ka+=e;const t=document.getElementById("wheel-calib-readout");if(!t)return;const a=s.scroll.detentsPerRev||60,n=Ka*a/4096;t.textContent=`Rueda en ${Kn(Gn()).toFixed(0)}°  ·  ${n>=0?"+":""}${n.toFixed(2)} clics en este giro`,clearTimeout(ei),ei=setTimeout(()=>{Ka=0},1200)}function Xa(e){Un(e),$a(),Ra()}function $a(){const e=document.getElementById("settings-wheel-calib-body");if(!e)return;const t=$e;e.innerHTML=`
    <p class="setting-desc">
      Cómo se dibuja la rueda magnética en pantalla. No afecta al teclado: solo hace que el
      dibujo coincida con el marcador que lleve pegado la tapa.
    </p>

    <div class="wheel-dial">
      <div class="wheel-dial-face">${Xn("settings-wheel-needle")}</div>
    </div>
    <p class="wheel-live-readout" id="wheel-calib-readout">Gira la rueda para comprobar el recorrido</p>

    <div class="dial-calib">
      <span class="field-label">Marcador en pantalla</span>

      <div class="chip-row">
        <button class="chip ${t.marker==="dot"?"on":""}" data-act="dial-marker" data-marker="dot">Círculo</button>
        <button class="chip ${t.marker==="line"?"on":""}" data-act="dial-marker" data-marker="line">Raya</button>
        <button class="chip ${t.invert?"on":""}" data-act="dial-invert"
                title="Si el dibujo gira al revés que la rueda">Invertir giro</button>
      </div>

      <div class="dial-offset">
        <button class="tool-btn small" data-act="dial-nudge" data-d="-1" title="1° menos">${f("minus",14)}</button>
        <input type="range" class="premium-slider" data-act="dial-offset"
               min="0" max="359" step="1" value="${Math.round(t.offsetDeg)}">
        <button class="tool-btn small" data-act="dial-nudge" data-d="1" title="1° más">${f("plus",14)}</button>
        <b id="dial-offset-val">${Math.round(t.offsetDeg)}°</b>
      </div>

      <button class="secondary-btn full" data-act="dial-align">
        ${f("fit",16)} El marcador está arriba: alinear aquí
      </button>
      <p class="setting-desc">
        Pon el circulito de la tapa mirando hacia arriba y pulsa el botón: el de la
        pantalla se coloca en el mismo sitio. Con la barra afinas grado a grado.
      </p>
    </div>`,as(Gn())}function xa(){kn(),En(),document.querySelectorAll("#timeout-selector .opt-btn").forEach(t=>{t.classList.toggle("active",Number(t.dataset.val)===s.timeout)});const e=document.getElementById("device-info-list");if(e){if(s.connected){const t=s.deviceInfo||{},a=Yi(t),n=a.level==="ok"?"var(--ok, inherit)":"var(--danger)";e.innerHTML=`
      <li><span class="lbl">Dispositivo</span><span class="val">${t.device||"ORBY_V4"}</span></li>
      <li><span class="lbl">Firmware</span><span class="val">${t.fw||"?"}</span></li>
      <li><span class="lbl">Compatibilidad</span><span class="val" style="color:${n}"
          title="${a.detail}">${a.level==="ok"?"al día":a.title}</span></li>
      <li><span class="lbl">Puerto</span><span class="val">${t.port||"—"}</span></li>
      <li><span class="lbl">Teclas / OLEDs</span><span class="val">${t.keys||12} / ${t.oleds||10}</span></li>
      <li><span class="lbl">Perfiles</span><span class="val">${s.profiles.length} / ${s.maxProfiles}</span></li>
      <li><span class="lbl">Modo</span><span class="val">${s.deviceMode}</span></li>
      <li><span class="lbl">Scroll alta res.</span><span class="val">${s.scroll.hires?"sí":"no"}</span></li>`}else e.innerHTML='<li><span class="lbl">Estado</span><span class="val" style="color:var(--danger)">Desconectado</span></li>';$a()}}const Xu=Object.freeze(Object.defineProperty({__proto__:null,init:ts,render:xa},Symbol.toStringTag,{value:"Module"})),ni=500,Ue=[];let Ct=!1,Ya=null,Pn="";function ns(){const e=document.getElementById("console-input");document.getElementById("btn-clear-console").addEventListener("click",()=>{Ue.length=0,Cn()});const t=document.getElementById("btn-pause-console");t.addEventListener("click",()=>{Ct=!Ct,t.classList.toggle("is-on",Ct),t.title=Ct?"Reanudar":"Pausar"}),document.getElementById("console-filter").addEventListener("input",r=>{Pn=r.target.value.trim().toUpperCase(),Cn()});const a=[];let n=-1;e.addEventListener("keydown",r=>{if(r.key==="Enter"&&e.value.trim()){const i=e.value.trim();a.unshift(i),n=-1,us(i),e.value=""}else r.key==="ArrowUp"&&a.length?(n=Math.min(n+1,a.length-1),e.value=a[n],r.preventDefault()):r.key==="ArrowDown"&&(n=Math.max(n-1,-1),e.value=n>=0?a[n]:"",r.preventDefault())}),W("rx",r=>Ve(r,"rx")),W("tx",r=>Ve(r,"tx")),W("error",r=>Ve(`ERROR: ${r}`,"error")),W("connected",r=>Ve(`Conectado: ${r?.raw||"ORBY_V4"}`,"system")),W("disconnected",()=>Ve("Dispositivo desconectado.","error")),Ve("Terminal OrbyGUI. Escribe GET_STATE para volcar la configuración.","system")}function Ve(e,t="system"){if(Ct&&t==="rx")return;const a=new Date().toLocaleTimeString("es-ES",{hour12:!1});Ue.push({time:a,text:e,type:t}),Ue.length>ni&&Ue.splice(0,Ue.length-ni),Cn()}function Cn(){Ya||(Ya=requestAnimationFrame(()=>{Ya=null,Qu()}))}function Yu(e){return e.replace(/[&<>]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;"})[t])}function Qu(){const e=document.getElementById("console-output");if(!e)return;const t=e.scrollHeight-e.scrollTop-e.clientHeight<40,a=Pn?Ue.filter(n=>n.text.toUpperCase().includes(Pn)):Ue;e.innerHTML=a.map(n=>`<div class="console-line ${n.type}"><span class="ts">${n.time}</span>${Yu(n.text)}</div>`).join(""),t&&(e.scrollTop=e.scrollHeight)}const Zu=Object.freeze(Object.defineProperty({__proto__:null,init:ns,push:Ve},Symbol.toStringTag,{value:"Module"})),ea={"view-dashboard":Ic,"view-profiles":Kd,"view-oled":Au,"view-auto":zu,"view-settings":Xu,"view-console":Zu};let ke="view-dashboard",rs=Promise.resolve(),ri=!1;const Ju=2500;function is(){ri||(ri=!0,Hi())}function St(e,t="info",a=900){Vl()?pa(e):g(e,t,a)}function ep(){V("windowChrome")||document.querySelector(".titlebar-controls")?.classList.add("hidden"),V("autoProfile")||document.querySelector('.nav-item[data-target="view-auto"]')?.classList.add("hidden"),V("appUpdate")||document.getElementById("btn-update")?.classList.add("hidden"),document.getElementById("btn-minimize").addEventListener("click",()=>window.orby.minimize()),document.getElementById("btn-maximize").addEventListener("click",()=>window.orby.maximize()),document.getElementById("btn-close").addEventListener("click",()=>window.orby.close()),document.querySelectorAll(".nav-item").forEach(e=>{e.addEventListener("click",()=>Sn(e.dataset.target))}),document.getElementById("btn-save-flash").addEventListener("click",os),tp()}function tp(){if(!V("appUpdate"))return;document.getElementById("btn-update").addEventListener("click",async()=>{const{status:t,newVersion:a}=re;t==="downloaded"&&confirm(`Se instalará OrbyGUI ${a}.

La app se cerrará y volverá a abrirse sola. ¿Continuar?`)&&await Fi()}),Vi(ii),Fl(),ii()}function ii(){if(!V("appUpdate"))return;const e=document.getElementById("btn-update"),t=e.querySelector(".update-label"),{status:a,newVersion:n,percent:r}=re;e.classList.toggle("hidden",a!=="downloading"&&a!=="downloaded"),e.classList.toggle("ready",a==="downloaded"),a==="downloading"?(t.textContent=`Descargando ${r}%`,e.title=`Bajando OrbyGUI ${n}`):a==="downloaded"&&(t.textContent=`Actualizar a ${n}`,e.title=`OrbyGUI ${n} descargada. Se instalará sola en cuanto se guarden los cambios en la Flash; haz clic para instalarla ahora`)}const ap={"view-oled":"view-profiles"};function Sn(e,t){ke=e;const a=ap[e]||e;document.querySelectorAll(".nav-item").forEach(n=>n.classList.toggle("active",n.dataset.target===a)),document.querySelectorAll(".view").forEach(n=>n.classList.toggle("active",n.id===e)),t&&e==="view-oled"?jo(t):ea[e]?.render?.()}async function os(){const e=document.getElementById("btn-save-flash");if(s.connected){Ge&&(clearTimeout(Ge),Ge=null),e.disabled=!0;try{await Cl(async()=>{await fs(),s.dirty=!1,te()})}catch{g("El teclado no confirmó el guardado, reintentando…","error"),ss()}finally{e.disabled=!1}}}const np=1500;let Ge=null;function ss(){Ge&&clearTimeout(Ge),Ge=setTimeout(()=>{Ge=null,os()},np)}function oi(){const e=document.getElementById("connection-status-badge"),t=e.querySelector(".status-text");if(s.connected){e.className="status-badge connected";const n=s.deviceInfo?.fw?` · fw ${s.deviceInfo.fw}`:"";t.textContent=s.syncing?"Sincronizando…":`Orby V4 conectado${n}`}else e.className="status-badge disconnected",t.textContent="Desconectado";document.getElementById("readonly-banner").classList.toggle("hidden",s.connected),document.body.classList.toggle("read-only",!s.connected),s.dirty&&s.connected&&ss();const a=document.getElementById("btn-save-flash");a.classList.toggle("has-changes",s.dirty),a.querySelector(".save-label").textContent=s.dirty?"Guardando…":"Guardado",a.disabled=!s.connected}const rp=20;function ip(e){if(!e){De(s.activeProfileIdx);return}il((t,a)=>{t%rp===0&&ra(),t===a&&St("Iconos descargados","info",1500)}).then(()=>{ea[ke]?.render?.(),ra()}).catch(t=>{console.error("No se pudieron precargar los iconos:",t),ra()})}async function op(e=3){for(let t=0;t<e;t++){const a=await ws();if(a?.all)return a.per;if(!In())return null;await new Promise(n=>setTimeout(n,400))}return null}async function sp(e){s.connected=!0,s.deviceInfo=e,te();const t=Yi(e);if(t.level==="blocked"){g(`${t.title}. ${t.detail}`,"error",9e3);return}t.level!=="ok"&&g(`${t.title}. ${t.detail}`,"error",9e3),await rs;const a=lt(e,"hash");try{let n=null;a&&(St("Comprobando si la copia del PC sigue valiendo…"),n=await op(),n||(console.warn("[sync] el teclado no ha dado las huellas: toca releerlo todo"),St("El teclado no ha dado las huellas: hay que releerlo todo","error",5e3)));const r=await fe({expected:n,iconOf:ye,onProgress:(i,o)=>St(`Leyendo perfil ${i+1} de ${o}…`)});for(const i of r)Ci(i);al(s.profiles.length),ea[ke]?.render?.(),r.length?St("Perfiles cargados desde el teclado","success",2600):pa("Todo al día")}catch(n){g(`No se pudo leer la configuración: ${n.message}`,"error");return}ip(a),io?.().catch(()=>{}),V("firmwareUpdate")&&Qi()?.then(n=>{n?.available&&g(`Hay firmware nuevo para el teclado (${n.latest.version}). Ajustes → Firmware del teclado`,"info",8e3)}).catch(()=>{})}function lp(){W("connected",e=>{sp(e).catch(t=>{console.error("Fallo al preparar la conexión con el teclado:",t)}).finally(is)}),W("disconnected",()=>{s.connected=!1,s.deviceInfo=null,te()}),W("telemetry",e=>{if(!e.startsWith("EV:CTX:"))return;const[t,a,n]=e.slice(7).split(":").map(Number);Os(t,a,n)&&(De(t),ea[ke]?.render?.()),Pl().catch(()=>{})}),W("searching",()=>{if(s.connected)return;const e=document.getElementById("connection-status-badge");e.className="status-badge searching",e.querySelector(".status-text").textContent="Buscando USB…"})}function si(){Hl(),ol(),ep(),Ps(),lp(),ll(Sn),cl().then(()=>{Ra(),ke==="view-profiles"&&P(),ke==="view-settings"&&xa()}),Kl(),Ec(),ml().then(()=>{ke==="view-profiles"&&P()}),Bl(),pa("Leyendo la copia del PC…"),rs=ql().then(e=>{e&&ea[ke]?.render?.(),pa(s.connected?"Hablando con el teclado…":"Buscando el teclado…"),setTimeout(()=>{s.connected||(is(),e&&g("Configuración cargada de la copia del PC; conecta el Orby para editarla","info",5e3))},Ju)}),lo(),Co(),Bo(),ts(),ns(),V("autoProfile")&&Zo(),it(()=>{oi(),xa(),ke==="view-dashboard"&&Ke()}),oi(),Sn("view-dashboard")}document.readyState==="loading"?document.addEventListener("DOMContentLoaded",si):si();
