import{c as H}from"./platform-_s-HGKbv.js";const wt=new Map;function W(e,t){return wt.has(e)||wt.set(e,new Set),wt.get(e).add(t),()=>wt.get(e).delete(t)}function we(e,t){const a=wt.get(e);if(a)for(const n of a)n(t)}let Ye=!1;function Sn(){return Ye}const ke=[];function os(e){for(let t=0;t<ke.length;t++){const a=ke[t];if(a.collect&&a.collect(e))return!0;if(a.match(e))return ke.splice(t,1),clearTimeout(a.timer),a.resolve(a.result!==void 0?a.result:e),!0;if(t===0&&a.fail&&a.fail(e))return ke.splice(t,1),clearTimeout(a.timer),a.reject(new Error(e)),!0}return!1}const oi="Conecta el Orby para poder editar la configuración",xa=20;function q(e,{match:t,collect:a,result:n,fail:r,timeout:i=4e3}={}){return Ye?(window.orby.sendCommand(e),we("tx",e),t?new Promise((o,c)=>{const u={match:t,collect:a,resolve:o,reject:c,result:n,fail:r};u.timer=setTimeout(()=>{const p=ke.indexOf(u);p>=0&&ke.splice(p,1),c(new Error(`Sin respuesta a "${e}"`))},i),ke.push(u)}):Promise.resolve(null)):Promise.reject(new Error(oi))}let Ya=null;function ss(e){Ya=e}function tt(e){return Ya?Ya(e):Promise.resolve()}function ls(e){window.orby.sendCommand(e),we("tx",e)}const si=e=>q(`SET_PROFILE:${e}`,{match:t=>t.startsWith("PROFILE:OK:")}),cs=e=>q(`SET_TIMEOUT:${e}`,{match:t=>t.startsWith("TIMEOUT:OK:")}),ds=()=>q("SAVE_STATE",{match:e=>e.startsWith("SAVE:OK"),timeout:8e3}),us=()=>q("RESET_DEFAULTS",{match:e=>e.startsWith("RESET:OK")}),li=async(e,t,a,n)=>(await tt(e),q(`SET_PSCROLL:${e}:${t}:${a}:${n?1:0}`,{match:r=>r.startsWith(`PSCROLL:OK:${e}:${t}:`)})),Ht=async(e,t,a)=>(await tt(e),q(`SET_LABEL:${e}:${t}:${a}`,{match:n=>n===`LABEL:OK:${e}:${t}`})),at=async(e,t,a,n)=>(await tt(e),q(`SET_KEYMAP:${e}:${t}:${a}:${n}`,{match:r=>r===`KEYMAP:OK:${e}:${t}`})),Vt=async(e,t,a,n,r)=>(await tt(e),q(`SET_ROTARY:${e}:${t}:${a}:${n}:${r}`,{match:i=>i===`ROTARY:OK:${e}:${t}`})),ci=(e,t)=>q(`SET_NAME:${e}:${t}`,{match:a=>a===`NAME:OK:${e}`}),Ft=async(e,t)=>(t!==xa&&await tt(e),q(`OLED_CLEAR:${e}:${t}`,{match:a=>a.startsWith(`OLED:CLEARED:${e}:`)})),In=e=>e.startsWith("ERR:"),ps=(e,t,a,n,r,i=1,o=0)=>q(`SET_MACRO_STEP:${e}:${t}:${a}:${n}:${r}:${i}:${o}`,{match:c=>c===`MACRO:OK:${e}:${t}`,fail:In}),fs=(e,t)=>q(`MACRO_TRUNC:${e}:${t}`,{match:a=>a===`MACRO:TRUNC:OK:${e}:${t}`,fail:In}),hs=e=>q(`MACRO_CLEAR:${e}`,{match:t=>t===`MACRO:CLEARED:${e}`,fail:In});function ms(e){const t=[],a=`MACRO:${e}:STEP:`;return q(`GET_MACRO:${e}`,{collect:n=>{if(!n.startsWith(a))return!1;const[r,i,o,c,u,p]=n.slice(a.length).split(":").map(Number);return t[r]={type:i,a:o,b:c,repeat:u,gap:p},!0},match:n=>n===`MACRO:${e}:END`||n.startsWith("ERR:"),result:t})}function Ln(e){return q(e,{timeout:6e3,match:t=>t.startsWith("PROFILE:ADDED:")||t.startsWith("PROFILE:DELETED:")||t.startsWith("ERR:")}).then(t=>{if(t.startsWith("ERR:"))throw new Error(t.slice(4));return parseInt(t.slice(t.lastIndexOf(":")+1),10)})}const di=()=>Ln("ADD_PROFILE"),gs=e=>Ln(`DUP_PROFILE:${e}`),ys=e=>Ln(`DEL_PROFILE:${e}`);function bs(){const e={};return q("GET_STATE",{collect:t=>{if(t.startsWith("STATE:PROFILE:"))return e.profile=parseInt(t.slice(14),10),!0;if(t.startsWith("STATE:PROFILES:")){const[a,n]=t.slice(15).split(":").map(Number);return e.profileCount=a,e.maxProfiles=n,!0}if(t.startsWith("STATE:TIMEOUT:"))return e.timeout=parseInt(t.slice(14),10),!0;if(t.startsWith("STATE:MODE:"))return e.mode=t.slice(11),!0;if(t.startsWith("STATE:SUPER:"))return e.superActive=t.slice(12)==="1",!0;if(t.startsWith("STATE:PAGE:")){const[a,n,r]=t.slice(11).split(":").map(Number);return e.pageIdx=a||0,e.pageCount=n||1,e.maxPages=r||1,!0}if(t.startsWith("SCROLL:OK:")){const[a,n,r,i]=t.slice(10).split(":").map(Number);return e.scroll={detentsPerRev:a,invert:!!n,hires:!!r,hiresPan:!!i},!0}return!1},match:t=>t==="STATE:END",result:e})}function vs(){const e={count:0,per:[],all:null};return q("GET_HASH",{timeout:8e3,collect:t=>{if(t.startsWith("HASH:PROFILES:"))return e.count=parseInt(t.slice(14),10),!0;if(t.startsWith("HASH:P:")){const[a,n]=t.slice(7).split(":");return e.per[parseInt(a,10)]=n,!0}return t.startsWith("HASH:ALL:")?(e.all=t.slice(9),!0):!1},match:t=>t==="HASH:END"||t.startsWith("ERR:"),result:e}).catch(()=>null)}function Qa(){const e={labels:new Array(20).fill(""),keys:[],rotary:[],scroll:[{detentsPerRev:60,invert:!1},{detentsPerRev:60,invert:!1}],oledMask:0};for(let t=0;t<24;t++)e.keys.push({modifier:0,keycode:0});for(let t=0;t<16;t++)e.rotary.push({type:0,modifier:0,keycode:0});return e}const $s=["labels","keys","rotary","scroll","oledMask"];function ui(e){for(const t of $s)Object.defineProperty(e,t,{get(){return(e.pages[e.pageIdx]||e.pages[0])[t]},set(a){(e.pages[e.pageIdx]||e.pages[0])[t]=a},enumerable:!1,configurable:!0});return e}function pi(e){const t={idx:e,name:"",pageCount:1,maxPages:1,pageIdx:0,pages:[]},a=Qa(),n=o=>{for(;t.pages.length<=o;)t.pages.push(Qa());return t.pages[o]},r=(o,c)=>{if(c.startsWith("LBL:")){const u=c.slice(4),p=u.indexOf(":");return o.labels[parseInt(u.slice(0,p),10)]=u.slice(p+1),!0}if(c.startsWith("KEY:")){const[u,p,m]=c.slice(4).split(":").map(Number);return o.keys[u]={modifier:p,keycode:m},!0}if(c.startsWith("ROT:")){const[u,p,m,v]=c.slice(4).split(":").map(Number);return o.rotary[u]={type:p,modifier:m,keycode:v},!0}if(c.startsWith("SCR:")){const[u,p,m]=c.slice(4).split(":").map(Number);return o.scroll[u]={detentsPerRev:p,invert:!!m},!0}return c.startsWith("OLEDMASK:")?(o.oledMask=parseInt(c.slice(9),10),!0):!1},i=`PROF:${e}:`;return q(`GET_PROFILE:${e}`,{timeout:12e3,collect:o=>{if(!o.startsWith(i))return!1;const c=o.slice(i.length);if(c.startsWith("NAME:"))return t.name=c.slice(5),!0;if(c.startsWith("PAGES:")){const[p,m]=c.slice(6).split(":").map(Number);return t.pageCount=p||1,t.maxPages=m||1,n(t.pageCount-1),!0}const u=c.match(/^P(\d+):(.*)$/);return u?r(n(parseInt(u[1],10)),u[2]):r(a,c)},match:o=>o===`${i}END`,result:t}).then(o=>(o.pages.length||(o.pages.push(a),o.pageCount=1,o.maxPages=1),ui(o)))}function fi(e){return q(`SET_PAGE:${e}`,{match:t=>t.startsWith("PAGE:OK:")||t.startsWith("ERR:")})}function hi(e,t=!0){return q(`ADD_PAGE:${e}:${t?1:0}`,{match:a=>a.startsWith("PAGE:ADDED:")||a.startsWith("ERR:")})}function xs(e,t){return q(`DEL_PAGE:${e}:${t}`,{match:a=>a.startsWith("PAGE:DELETED:")||a.startsWith("ERR:")})}async function An(e,t){const a=new Uint8Array(360);let n=!1;const r=`OLEDDATA:${e}:${t}:`;return t!==xa&&await tt(e),q(`GET_OLED:${e}:${t}`,{timeout:6e3,collect:i=>{if(!i.startsWith(r))return!1;const o=i.slice(r.length);if(o==="NONE"||o==="END")return!1;const c=o.indexOf(":");if(c<0)return!1;const u=parseInt(o.slice(0,c),10),p=o.slice(c+1);for(let m=0;m+1<p.length;m+=2)a[u+m/2]=parseInt(p.substr(m,2),16);return n=!0,!0},match:i=>i===`${r}END`||i===`${r}NONE`,result:null}).then(i=>n?a:null,()=>null)}function Tn(e,t,a){const n=new Uint8Array(360);let r=!1;const i=`OLEDDATA:${e}:P${t}:${a}:`;return q(`GET_OLED_PG:${e}:${t}:${a}`,{timeout:6e3,collect:o=>{if(!o.startsWith(i))return!1;const c=o.slice(i.length);if(c==="NONE"||c==="END")return!1;const u=c.indexOf(":");if(u<0)return!1;const p=parseInt(c.slice(0,u),10),m=c.slice(u+1);for(let v=0;v+1<m.length;v+=2)n[p+v/2]=parseInt(m.substr(v,2),16);return r=!0,!0},match:o=>o===`${i}END`||o===`${i}NONE`||o.startsWith("ERR:"),result:null}).then(()=>r?n:null,()=>null)}const mr=90;async function ve(e,t,a){t!==xa&&await tt(e);for(let n=0;n<a.length;n+=mr){const r=a.subarray(n,Math.min(n+mr,a.length));let i="";for(const o of r)i+=o.toString(16).padStart(2,"0");await q(`OLED_CHUNK:${e}:${t}:${n}:${i}`,{match:o=>o.startsWith(`OLED:OK:${e}:${t}:${n}:`)})}}let Za=null;function mi(e){const t=`${e?.port||""}|${e?.raw||""}`;Ye&&t===Za||(Za=t,Ye=!0,we("connected",e||{}))}const ws=2e3;let gr=0;function yr(){const e=Date.now();e-gr<ws||(gr=e,window.orby.getStatus().then(async t=>{t!=="connected"||Ye||mi(await window.orby.getDeviceInfo())}).catch(()=>{}))}function Ms(){window.orby.onConnected(e=>mi(e)),yr(),window.orby.onDisconnected(()=>{for(Ye=!1,Za=null;ke.length;){const e=ke.pop();clearTimeout(e.timer),e.reject(new Error(oi))}we("disconnected")}),window.orby.onSearching(()=>we("searching")),window.orby.onError(e=>we("error",e)),window.orby.onData(e=>{Ye||yr(),we("rx",e);const t=os(e);we(t?"response":"telemetry",e)})}const ks=4294967295,br=new Uint32Array([0,498536548,997073096,651767980,1994146192,1802195444,1303535960,1342533948,3988292384,4027552580,3604390888,3412177804,2607071920,2262029012,2685067896,3183342108]),Es=new TextEncoder;class Ps{constructor(){this.value=ks}bytes(t){let a=this.value;for(let n=0;n<t.length;n++)a=(a^t[n])>>>0,a=(a>>>4^br[a&15])>>>0,a=(a>>>4^br[a&15])>>>0;return this.value=a,this}u8(...t){return this.bytes(Uint8Array.from(t,a=>a&255))}u32le(t){return this.u8(t,t>>>8,t>>>16,t>>>24)}text8(t){const a=new Uint8Array(8);return a.set(Es.encode(String(t??"")).subarray(0,8)),this.bytes(a)}done(){return(this.value^4294967295)>>>0}}function vr(e){return(e>>>0).toString(16).padStart(8,"0")}function Cs(e,t){if(!e||!e.pages?.length)return null;const a=Math.max(1,Math.min(e.pageCount||1,e.pages.length)),n=new Ps;n.text8(e.name),n.u8(a);for(let i=0;i<a;i++){const o=e.pages[i];if(!o)return null;for(let u=0;u<20;u++)n.text8(o.labels?.[u]??"");for(let u=0;u<24;u++){const p=o.keys?.[u]||{modifier:0,keycode:0};n.u8(p.modifier||0,p.keycode||0)}for(let u=0;u<16;u++){const p=o.rotary?.[u]||{type:0,modifier:0,keycode:0};n.u8(p.type||0,p.modifier||0,p.keycode||0)}for(let u=0;u<2;u++){const p=o.scroll?.[u]||{detentsPerRev:60,invert:!1};n.u8(p.detentsPerRev||0,p.invert?1:0)}const c=(o.oledMask||0)&1048575;n.u32le(c);for(let u=0;u<20;u++){if(!(c&1<<u))continue;const p=t(u,i);if(!p||p.length!==360)return null;n.bytes(p)}}const r=(e.pages[0]?.oledMask||0)&1<<20?1:0;if(n.u8(r),r){const i=t(20,0);if(!i||i.length!==360)return null;n.bytes(i)}return n.done()}const Ja=new Set,s={connected:!1,deviceInfo:null,activeProfileIdx:0,maxProfiles:8,deviceMode:"NORMAL",superActive:!1,timeout:5,scroll:{detentsPerRev:60,invert:!1,hires:!1},profiles:[],dirty:!1,syncing:!1,pageIdx:0,pageCount:1,maxPages:1};function gi(e,t=s.pageIdx){return!e||!e.pages?null:e.pages[t]||e.pages[0]||null}function D(e){return e&&e.pages?Math.max(1,e.pageCount||e.pages.length):1}function Wt(){return Math.max(1,s.maxPages||1)}function nt(){return Wt()>1}const $r=[{icon:"profiles",accent:"#8b5cf6"},{icon:"pencil",accent:"#ec4899"},{icon:"bolt",accent:"#22d3ee"},{icon:"oled",accent:"#f59e0b"},{icon:"key",accent:"#10b981"},{icon:"wheel",accent:"#f43f5e"},{icon:"text",accent:"#6366f1"},{icon:"fill",accent:"#84cc16"}];function sa(e){return $r[e%$r.length]}function rt(e){return Ja.add(e),()=>Ja.delete(e)}function te(){for(const e of Ja)try{e(s)}catch(t){console.error("Error en un suscriptor de store.notify():",t)}}function z(){s.dirty=!0,te()}function bt(e=s.activeProfileIdx){return s.profiles[e]||null}const _e=[1,2,3,4,5,6,7,8,9,0,10,0],ft=12,en=10;function yi(e,t,a){const n=_e[t];return!n||!e?null:e.labels[n-1+(a==="super"?10:0)]||""}function V(e,t){const a=_e[e];return a?a-1+(t==="super"?10:0):-1}function le(e,t){return e+(t==="super"?12:0)}const Ss=8;function pe(e,t){return e+(t==="super"?Ss:0)}function bi(e){return e==="super"?1:0}function Rn(e,t){return e?.scroll?.[bi(t)]||{detentsPerRev:60,invert:!1}}function Is(e,t,a){const n=Math.max(1,Math.min(e.pageCount||1,e.pages.length));for(let r=0;r<n;r++){const i=(e.pages[r]?.oledMask||0)&1048575;for(let o=0;o<20;o++){if(!(i&1<<o))continue;const c=a(t,o,r);if(!c||c.length!==360)return`${t}:${r}:${o}`}}return"(ninguno: la huella falla por otra cosa)"}function Ls(e,t,a,n){if(!e||!a?.[t]||!n)return null;const r=Cs(e,(i,o)=>n(t,i,o));return r===null||vr(r)!==a[t]?(console.debug(`[sync] perfil ${t} hay que releerlo:`,r===null?`falta el icono ${Is(e,t,n)}`:`${vr(r)} != ${a[t]}`),null):(e.idx=t,e)}async function fe({expected:e=null,iconOf:t=null,onProgress:a=null}={}){s.syncing=!0,te();const n=[];try{const r=await bs();let i=s.profiles.length||4;r&&(Number.isInteger(r.profile)&&(s.activeProfileIdx=r.profile),Number.isInteger(r.profileCount)&&(i=r.profileCount),Number.isInteger(r.maxProfiles)&&(s.maxProfiles=r.maxProfiles),Number.isInteger(r.timeout)&&(s.timeout=r.timeout),typeof r.superActive=="boolean"&&(s.superActive=r.superActive),r.mode&&(s.deviceMode=r.mode),r.scroll&&(s.scroll=r.scroll),Number.isInteger(r.pageIdx)&&(s.pageIdx=r.pageIdx),Number.isInteger(r.pageCount)&&(s.pageCount=r.pageCount),Number.isInteger(r.maxPages)&&(s.maxPages=r.maxPages));const o=s.profiles,c=[];for(let u=0;u<i;u++){const p=Ls(o[u],u,e,t);if(p){c.push(p);continue}a?.(u,i),c.push(await pi(u)),n.push(u)}s.profiles=c,s.activeProfileIdx>=c.length&&(s.activeProfileIdx=0);for(const u of c)u.pageIdx=u.idx===s.activeProfileIdx?Math.min(s.pageIdx,D(u)-1):0;s.pageCount=D(c[s.activeProfileIdx]),s.dirty=!1}finally{s.syncing=!1,te()}return n}async function vi(e){const t=bt(),a=D(t);if(!(e<0||e>=a)&&(t&&(t.pageIdx=e),s.pageIdx=e,te(),!!Sn()))try{await fi(e)}catch(n){throw await fe(),n}}async function Qe(e,t,a=!0){if(!Sn())return;const n=s.profiles[e];if(!n)return;const r=Math.min(Math.max(t||0,0),D(n)-1);if(e!==s.activeProfileIdx){if(r!==0)throw new Error("Activa este perfil en el teclado para editar esta página");return}s.pageIdx!==r&&(await fi(r),s.pageIdx=r,a&&(n.pageIdx=r))}let dt=null;async function Nn(e,t,a){if(dt)return a();const n=s.pageIdx;dt={profile:e,page:t};try{return await Qe(e,t,!1),await a()}finally{if(dt=null,s.pageIdx!==n)try{await Qe(e,n,!1)}catch{}}}function As(e){if(dt&&dt.profile===e)return Qe(e,dt.page,!1);const t=s.profiles[e];return Qe(e,t&&t.pageIdx||0)}ss(As);function Ts(e,t,a){if(!Number.isInteger(e)||!Number.isInteger(t)||s.activeProfileIdx===e&&s.pageIdx===t&&s.pageCount===a)return!1;const n=s.profiles[s.activeProfileIdx],r=s.profiles[e];return s.activeProfileIdx=e,s.pageIdx=t,Number.isInteger(a)&&a>0&&(s.pageCount=a),n&&n!==r&&(n.pageIdx=0),r&&(Number.isInteger(a)&&a>0&&(r.pageCount=a),r.pageIdx=Math.min(Math.max(t,0),D(r)-1)),te(),!0}async function Rs(){const e=bt();return!e||D(e)>=Wt()?!1:(await hi(e.idx,!1),await fe(),z(),!0)}async function Ns(e){const t=bt();return!t||D(t)<=1?!1:(await xs(t.idx,e),await fe(),z(),!0)}function $i(){const e=bt();return{...Rn(e,s.superActive?"super":"normal"),hires:s.scroll.hires}}const w=72,I=40,xi=I/8,Os=w*xi;function Ut(){return new Uint8Array(Os)}function ht(e,t,a){return t<0||a<0||t>=w||a>=I?0:e[(a>>3)*w+t]>>(a&7)&1}function ne(e,t,a,n){if(t<0||a<0||t>=w||a>=I)return;const r=(a>>3)*w+t,i=1<<(a&7);n?e[r]|=i:e[r]&=~i}function qs(e){for(let t=0;t<e.length;t++)e[t]=~e[t]&255}function wi(e){e.fill(0)}function _s(e,t,a,n){const r=ht(e,t,a);if(r===n)return;const i=[[t,a]];for(;i.length;){const[o,c]=i.pop();o<0||c<0||o>=w||c>=I||ht(e,o,c)===r&&(ne(e,o,c,n),i.push([o+1,c],[o-1,c],[o,c+1],[o,c-1]))}}function zs(e,t,a,n,r,i){const o=Math.abs(n-t),c=t<n?1:-1,u=-Math.abs(r-a),p=a<r?1:-1;let m=o+u;for(;ne(e,t,a,i),!(t===n&&a===r);){const v=2*m;v>=u&&(m+=u,t+=c),v<=o&&(m+=o,a+=p)}}function Mi(e){for(let t=0;t<w;t++)ne(e,t,0,1),ne(e,t,I-1,1);for(let t=0;t<I;t++)ne(e,0,t,1),ne(e,w-1,t,1)}function Bs(e,t){if(t<=0)return e;const a=new Float32Array(e.length),n=new Float32Array(e.length);for(let r=0;r<I;r++)for(let i=0;i<w;i++){let o=0,c=0;for(let u=-t;u<=t;u++){const p=i+u;p<0||p>=w||(o+=e[r*w+p],c++)}a[r*w+i]=o/c}for(let r=0;r<w;r++)for(let i=0;i<I;i++){let o=0,c=0;for(let u=-t;u<=t;u++){const p=i+u;p<0||p>=I||(o+=a[p*w+r],c++)}n[i*w+r]=o/c}return n}function js(e,{threshold:t=128,dither:a=!1,blur:n=0,invert:r="none",bounds:i=null}={}){const{data:o}=e.getImageData(0,0,w,I),c=Ut(),u=r==="colors";let p=new Float32Array(w*I);for(let m=0;m<w*I;m++){const v=m*4,y=o[v+3]/255,h=.299*o[v]+.587*o[v+1]+.114*o[v+2];p[m]=(u?255-h:h)*y}if(p=Bs(p,n),r==="box"&&i){const m=Math.max(0,Math.floor(i.x)),v=Math.max(0,Math.floor(i.y)),y=Math.min(w,Math.ceil(i.x+i.width)),h=Math.min(I,Math.ceil(i.y+i.height));for(let k=v;k<h;k++)for(let B=m;B<y;B++){const F=k*w+B;p[F]=255-p[F]}}if(a)for(let m=0;m<I;m++)for(let v=0;v<w;v++){const y=m*w+v,h=p[y],k=h<t?0:255;p[y]=k;const B=h-k;v+1<w&&(p[y+1]+=B*7/16),v>0&&m+1<I&&(p[y+w-1]+=B*3/16),m+1<I&&(p[y+w]+=B*5/16),v+1<w&&m+1<I&&(p[y+w+1]+=B*1/16)}for(let m=0;m<I;m++)for(let v=0;v<w;v++)ne(c,v,m,p[m*w+v]>=t?1:0);return c}function On(){const e=document.createElement("canvas");return e.width=w,e.height=I,e.getContext("2d",{willReadFrequently:!0})}async function Ds(e){const t=await createImageBitmap(e);return{kind:"image",bitmap:t,naturalWidth:t.width,naturalHeight:t.height}}const Hs=256;function ki(e,{size:t=Hs}={}){return new Promise((a,n)=>{const r=new Image;r.width=t,r.height=t,r.onload=()=>a({kind:"image",bitmap:r,naturalWidth:t,naturalHeight:t,crisp:!0}),r.onerror=()=>n(new Error("SVG no válido")),r.src=`data:image/svg+xml;charset=utf-8,${encodeURIComponent(e)}`})}function qn(e,{fontSize:t=16,bold:a=!0,font:n="Segoe UI"}={}){return{kind:"text",text:String(e),fontSize:t,bold:a,font:n}}function _n(e){return`${e.bold?"700 ":""}${e.fontSize}px "${e.font}", sans-serif`}function zn(e){return e.text.split(`
`).slice(0,4)}function Bn(e){if(e.kind==="image")return{width:e.naturalWidth,height:e.naturalHeight};const t=On();t.font=_n(e);const a=zn(e);return{width:Math.max(1,...a.map(r=>t.measureText(r).width)),height:a.length*(e.fontSize+2)}}function it(e){return e._ink||(e._ink=e.kind==="text"?Vs(e):Ws(e)),e._ink}function Vs(e){const t=On();t.font=_n(e),t.textAlign="left",t.textBaseline="top";const a=e.fontSize+2;let n=1/0,r=1/0,i=-1/0,o=-1/0;if(zn(e).forEach((c,u)=>{if(!c.trim())return;const p=t.measureText(c),m=u*a;n=Math.min(n,-(p.actualBoundingBoxLeft||0)),i=Math.max(i,p.actualBoundingBoxRight??p.width),r=Math.min(r,m-(p.actualBoundingBoxAscent||0)),o=Math.max(o,m+(p.actualBoundingBoxDescent||0))}),!(i>n)||!(o>r)){const{width:c,height:u}=Bn(e);return{x:0,y:0,width:c,height:u}}return{x:n,y:r,width:i-n,height:o-r}}const Fs=512;function Ws(e){const t=e.naturalWidth,a=e.naturalHeight,n={x:0,y:0,width:t,height:a},r=Math.min(1,Fs/Math.max(t,a)),i=Math.max(1,Math.round(t*r)),o=Math.max(1,Math.round(a*r)),c=document.createElement("canvas");c.width=i,c.height=o;const u=c.getContext("2d",{willReadFrequently:!0});u.drawImage(e.bitmap,0,0,i,o);let p;try{p=u.getImageData(0,0,i,o).data}catch{return n}let m=i,v=o,y=-1,h=-1;for(let F=0;F<o;F++)for(let E=0;E<i;E++)p[(F*i+E)*4+3]<8||(E<m&&(m=E),E>y&&(y=E),F<v&&(v=F),F>h&&(h=F));if(y<0)return n;const k=t/i,B=a/o;return{x:m*k,y:v*B,width:(y-m+1)*k,height:(h-v+1)*B}}function jn(e){const t=it(e);return!t.width||!t.height?1:Math.min((w-2)/t.width,(I-2)/t.height)}function Us(e,t,a){const{width:n,height:r}=Bn(t),i=n*a.scale,o=r*a.scale;if(t.kind==="image")return e.imageSmoothingEnabled=!0,e.imageSmoothingQuality="high",e.drawImage(t.bitmap,a.x,a.y,i,o),{x:a.x,y:a.y,width:i,height:o};e.save(),e.fillStyle="#fff",e.textAlign="left",e.textBaseline="top",e.translate(a.x,a.y),e.scale(a.scale,a.scale),e.font=_n(t);const c=t.fontSize+2;return zn(t).forEach((u,p)=>e.fillText(u,0,p*c)),e.restore(),{x:a.x,y:a.y,width:i,height:o}}function wa(e,t){const a=it(e);return{x:t.x+a.x*t.scale,y:t.y+a.y*t.scale,width:a.width*t.scale,height:a.height*t.scale}}function Gt(e,t){const a=On();return Us(a,e,t),js(a,{threshold:t.threshold,dither:t.dither,blur:t.blur,invert:t.invert,bounds:wa(e,t)})}function Gs(e){let t=w,a=I,n=-1,r=-1;for(let i=0;i<I;i++)for(let o=0;o<w;o++)ht(e,o,i)&&(o<t&&(t=o),o>n&&(n=o),i<a&&(a=i),i>r&&(r=i));return n<0?null:{x:t,y:a,width:n-t+1,height:r-a+1}}function Dn(e,t,a="replace"){const n=Ut();for(let r=0;r<n.length;r++)n[r]=a==="merge"?e[r]|t[r]:t[r];return n}function tn(e,t,{zoom:a=8,grid:n=!0,color:r="#e9e2ff",outline:i=null}={}){t.width=w*a,t.height=I*a;const o=t.getContext("2d");o.fillStyle="#05050a",o.fillRect(0,0,t.width,t.height),o.fillStyle=r;for(let c=0;c<I;c++)for(let u=0;u<w;u++)ht(e,u,c)&&o.fillRect(u*a,c*a,a,a);if(n&&a>=5){o.strokeStyle="rgba(255,255,255,0.07)",o.lineWidth=1,o.beginPath();for(let c=0;c<=w;c++)o.moveTo(c*a+.5,0),o.lineTo(c*a+.5,t.height);for(let c=0;c<=I;c++)o.moveTo(0,c*a+.5),o.lineTo(t.width,c*a+.5);o.stroke(),o.strokeStyle="rgba(139,92,246,0.25)",o.beginPath();for(let c=1;c<xi;c++)o.moveTo(0,c*8*a+.5),o.lineTo(t.width,c*8*a+.5);o.stroke()}if(i){o.save(),o.strokeStyle="#f59e0b",o.lineWidth=2,o.setLineDash([6,4]),o.strokeRect(i.x*a,i.y*a,i.width*a,i.height*a),o.setLineDash([]),o.fillStyle="#f59e0b";const c=Math.max(5,Math.min(10,a)),u=[[i.x,i.y],[i.x+i.width,i.y],[i.x,i.y+i.height],[i.x+i.width,i.y+i.height]];for(const[p,m]of u)o.fillRect(p*a-c/2,m*a-c/2,c,c);o.restore()}}const j=new Map,an=new Set;let Ct=null,De=null,ta=!1;function Ks(e){const t=s.profiles[e];return t&&t.pageIdx||0}function ue(e,t,a=Ks(e)){return`${e}:${a}:${t}`}function ye(e,t,a){return j.get(ue(e,t,a))??null}function Hn(e){return ye(e,20,0)}function ze(e,t,a,n){j.set(ue(e,t,n),a),se()}function Xs(e){let t="";for(const a of e)t+=a.toString(16).padStart(2,"0");return t}function Ys(e){const t=new Uint8Array(e.length/2);for(let a=0;a<t.length;a++)t[a]=parseInt(e.substr(a*2,2),16);return t}function Qs(){const e={};for(const[t,a]of j)a&&(e[t]=Xs(a));return e}function Zs(e){for(const[t,a]of Object.entries(e||{}))typeof a=="string"&&a.length&&j.set(t,Ys(a));se()}function Ma(){j.clear(),Ct=null,De=null,se()}function Ei(e){const t=`${e}:`;for(const a of[...j.keys()])a.startsWith(t)&&j.delete(a);se()}function Js(e){for(const t of[...j.keys()])parseInt(t,10)>=e&&j.delete(t);se()}function el(){return Ct!==null||ta}function Vn(e){return an.add(e),()=>an.delete(e)}function se(){for(const e of an)e()}async function je(e){if(s.connected){if(Ct!==null){De=e;return}Ct=e,se();try{const t=s.profiles[e];if(!t)return;const a=t.pageIdx||0,n=()=>s.profiles[e]!==t||(t.pageIdx||0)!==a;for(let i=0;i<20;i++){if(n()){De=e;return}if(j.has(ue(e,i,a)))continue;if(!(t.oledMask&1<<i)){j.set(ue(e,i,a),null);continue}const o=await An(e,i);if(n()){De=e;return}j.set(ue(e,i,a),o),se()}const r=ue(e,20,0);if(!j.has(r))if(!((t.pages[0]?.oledMask||0)&1<<20))j.set(r,null);else{const i=await Tn(e,0,20);n()||(j.set(r,i),se())}}finally{if(Ct=null,se(),De!==null){const t=De;De=null,je(t)}}}}const tl=10;async function al(e=()=>{}){if(ta)return;const t=[];for(const n of s.profiles){const r=D(n);for(let o=0;o<r;o++){const c=n.pages?.[o]?.oledMask||0;for(let u=0;u<20;u++){const p=ue(n.idx,u,o);if(!j.has(p)){if(!(c&1<<u)){j.set(p,null);continue}t.push({profile:n.idx,page:o,slot:u,key:p})}}}const i=ue(n.idx,20,0);j.has(i)||((n.pages[0]?.oledMask||0)&1<<20?t.push({profile:n.idx,page:0,slot:20,key:i}):j.set(i,null))}if(se(),!t.length)return;ta=!0;let a=0;try{for(const n of t){if(!s.connected)return;j.set(n.key,await Tn(n.profile,n.page,n.slot)),a++,a%tl===0&&se(),e(a,t.length)}}finally{ta=!1,se()}}function Kt(e=document){e.querySelectorAll("[data-bmp]").forEach(t=>{const a=j.get(t.dataset.bmp);a&&tn(a,t,{zoom:1,grid:!1})})}const xr={dashboard:'<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',profiles:'<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M9 9v12"/>',wheel:'<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="2.4" fill="currentColor" stroke="none"/>',oled:'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M6 15l3.5-4 2.5 3 2-2.5L18 15"/>',settings:'<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2"/><circle cx="15" cy="12" r="2"/><circle cx="8" cy="18" r="2"/>',console:'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>',minus:'<path d="M5 12h14"/>',plus:'<path d="M12 5v14M5 12h14"/>',fit:'<path d="M3 8V4h4M21 8V4h-4M3 16v4h4M21 16v4h-4"/><rect x="8" y="9" width="8" height="6" rx="1"/>',up:'<path d="M6 15l6-6 6 6"/>',down:'<path d="M6 9l6 6 6-6"/>',square:'<rect x="5" y="5" width="14" height="14" rx="1"/>',select:'<rect x="4" y="4" width="16" height="16" rx="1" stroke-dasharray="3 3"/>',close:'<path d="M6 6l12 12M18 6L6 18"/>',save:'<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>',refresh:'<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>',trash:'<path d="M3 6h18M8 6V4h8v2M6 6l1 14h10l1-14"/>',upload:'<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 3v13M7 8l5-5 5 5"/>',pencil:'<path d="M17 3l4 4L8 20l-5 1 1-5z"/>',eraser:'<path d="M4 16L13 7l6 6-6 6H7z"/><path d="M9 21h11"/>',fill:'<path d="M4 12L12 4l8 8-8 8z"/><path d="M19 15c1.5 2 2 2.8 2 3.6a2 2 0 0 1-4 0c0-.8.5-1.6 2-3.6z"/>',invert:'<circle cx="12" cy="12" r="9"/><path d="M12 3a9 9 0 0 1 0 18z" fill="currentColor" stroke="none"/>',text:'<path d="M4 6V4h16v2M12 4v16M8 20h8"/>',check:'<path d="M4 12l5 5L20 6"/>',key:'<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>',lock:'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/>',bolt:'<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>',sun:'<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',moon:'<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>',info:'<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',plug:'<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>',reset:'<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>',download:'<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 16V3M7 11l5 5 5-5"/>',copy:'<rect x="8" y="8" width="12" height="13" rx="1.5"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>',paste:'<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>'};function f(e,t=20){const a=xr[e]||xr.info;return`<svg class="icn" width="${t}" height="${t}" viewBox="0 0 24 24" fill="none"
    stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"
    aria-hidden="true">${a}</svg>`}function nl(e=document){e.querySelectorAll("[data-icon]").forEach(t=>{const a=parseInt(t.getAttribute("data-icon-size")||"20",10);t.innerHTML=f(t.getAttribute("data-icon"),a),t.removeAttribute("data-icon")})}const rl={success:"check",error:"close",info:"info"};function g(e,t="success",a=2600){let n=document.getElementById("toast-host");n||(n=document.createElement("div"),n.id="toast-host",document.body.appendChild(n));const r=document.createElement("div");r.className=`toast toast-${t}`,r.innerHTML=`${f(rl[t]||"info",16)}<span>${e}</span>`,n.appendChild(r),setTimeout(()=>{r.classList.add("leaving"),setTimeout(()=>r.remove(),200)},a)}function Y(){return s.connected?!0:(g("Conecta el Orby por USB para poder editar la configuración","error",4e3),!1)}let Pi=null;function il(e){Pi=e}function nn(e,t){Pi?.(e,t)}const St=4096,$e={invert:!0,offsetDeg:62,marker:"dot"};let Tt=null,Fn=0,aa=0;const rn=new Set;function Ci(e){return rn.add(e),()=>rn.delete(e)}async function ol(){try{const e=await window.orby.getConfig();Object.assign($e,e.wheelDial||{})}catch{}W("telemetry",e=>{e.startsWith("WHEEL:")&&sl(e)})}function Wn(e){Object.assign($e,e),window.orby.setConfig({wheelDial:{...$e}}),Si()}function sl(e){const[t,a]=e.slice(6).split(":"),n=parseInt(t,10);if(a!==void 0){const r=parseInt(a,10);if(!Number.isFinite(r))return;Tt=ll(r)/St*360}else if(Number.isFinite(n))Fn+=n/St*360;else return;Si()}function ll(e){return(e%St+St)%St}function cl(){const e=Tt!==null?Tt:Fn;return($e.invert?e:-e)+$e.offsetDeg}function Si(){let e=(cl()-aa)%360;if(Number.isFinite(e)){e>180&&(e-=360),e<-180&&(e+=360),aa+=e;for(const t of rn)t(aa)}}function Un(){return aa}function dl(){const e=Tt!==null?Tt:Fn,t=$e.invert?e:-e;Wn({offsetDeg:Gn(-t)})}function Gn(e){return(e%360+360)%360}function Kn(e){return`<div class="dial-marker ${$e.marker==="line"?"as-line":"as-dot"}" id="${e}"></div>`}const X=[];let _=null,Ii=null,Li=!1;const on=new Set;function Ai(e){return on.add(e),()=>on.delete(e)}function ce(e="data"){for(const t of on)t(e)}function Le(){return window.orby.setConfig({profileVariants:X})}function ul(e){return Array.isArray(e.matches)||(e.matches=e.match?[e.match]:[]),delete e.match,e.keys=e.keys||{},e.rotary=e.rotary||{},e.labels=e.labels||{},Number.isInteger(e.page)||(e.page=0),e}async function pl(){try{const e=await window.orby.getConfig();X.push(...(e.profileVariants||[]).map(ul))}catch{}W("disconnected",()=>{_=null,ce("applied")})}function fl(e,t){return X.filter(a=>a.profile===e&&(t===void 0||a.page===t))}function ot(e){return X.find(t=>t.id===e)||null}function hl(){return _?.v||null}function Ae(e){return _?.v.id===e}function he(e,t,a){return e?.[t]?.[a]??null}function Rt(e){return e?Object.keys(e.keys||{}).length+Object.keys(e.rotary||{}).length+Object.keys(e.labels||{}).length:0}function ml(e,{page:t=0,name:a,matches:n=[],field:r="any"}={}){const i={id:`v${Date.now()}${Math.random().toString(36).slice(2,6)}`,profile:e,page:t,name:a||"Variación",matches:n.filter(Boolean),field:r,keys:{},rotary:{},labels:{}};return X.push(i),Le(),ce(),i}function Ti(e,t){const a=ot(e),n=(t||"").trim().toLowerCase();return!a||!n||a.matches.includes(n)?!1:(a.matches.push(n),Le(),ce(),!0)}function gl(e,t){const a=ot(e);if(!a)return;const n=a.matches.indexOf(t);n>=0&&a.matches.splice(n,1),Le(),ce()}function wr(e,t){const a=ot(e);a&&(Object.assign(a,t),Le(),ce())}async function yl(e){Ae(e)&&await vt();const t=X.findIndex(a=>a.id===e);t>=0&&X.splice(t,1),Le(),ce()}function Xn(e,t,a,n){const r=ot(e);r&&(r[t]=r[t]||{},r[t][a]=n,Le(),ce())}function Mr(e,t,a){const n=ot(e);n?.[t]&&(delete n[t][a],Le(),ce())}function bl(e,t){for(let a=X.length-1;a>=0;a--){const n=X[a];n.profile===e&&(n.page===t?X.splice(a,1):n.page>t&&n.page--)}Le(),ce()}function vl(e){for(let t=X.length-1;t>=0;t--)X[t].profile===e?X.splice(t,1):X[t].profile>e&&X[t].profile--;Le(),ce()}function $l(e){Ii=e}function sn(e){Li=e}function xl(e,t){return e?t==="title"?(e.title||"").toLowerCase():t==="process"?(e.process||"").toLowerCase():`${e.process||""} ${e.title||""}`.toLowerCase():""}function wl(e,t,a){for(const n of X){if(n.profile!==e||n.page!==t)continue;const r=xl(a,n.field);for(const i of n.matches||[])if(i&&r.includes(i))return n}return null}async function Ri(e,t,a){const n=s.profiles[e.profile];if(!n)return;const r=gi(n,t);r&&await Nn(e.profile,t,async()=>{for(const[i,o]of Object.entries(e.keys||{})){const c=a?r.keys[i]||{modifier:0,keycode:0}:o;await at(e.profile,Number(i),c.modifier,c.keycode)}for(const[i,o]of Object.entries(e.rotary||{})){const c=a?r.rotary[i]||{type:0,modifier:0,keycode:0}:o;await Vt(e.profile,Number(i),c.type,c.modifier,c.keycode)}for(const[i,o]of Object.entries(e.labels||{})){const c=a?r.labels[i]||"":o;await Ht(e.profile,Number(i),c)}})}function Nt(){return s.pageIdx||0}async function Ni(e){const t=ot(e);if(!t||!s.connected||t.profile!==s.activeProfileIdx||t.page!==Nt())return;_&&_.v.id!==t.id&&await vt();const a=t.page;try{await Ri(t,a,!1),_={v:t,page:a},ce("applied")}catch{}}async function ka(e,t){if(!_||_.page!==Nt())return;const{v:a,page:n}=_,r=a[e]?.[t];if(r)try{await Nn(a.profile,n,async()=>{e==="keys"&&await at(a.profile,Number(t),r.modifier,r.keycode),e==="rotary"&&await Vt(a.profile,Number(t),r.type,r.modifier,r.keycode),e==="labels"&&await Ht(a.profile,Number(t),r)})}catch{}}async function Ea(e,t,a,n){if(!Ae(e)||!s.connected)return;const{v:r,page:i}=_;await Nn(r.profile,i,async()=>{t==="keys"&&await at(r.profile,Number(a),n.modifier,n.keycode),t==="rotary"&&await Vt(r.profile,Number(a),n.type,n.modifier,n.keycode),t==="labels"&&await Ht(r.profile,Number(a),n)})}async function _a(e,t,a){if(!Ae(e)||!s.connected)return;const n=s.profiles[_.v.profile],r=n&&gi(n,_.page);if(!r)return;const i=t==="keys"?{modifier:0,keycode:0}:t==="rotary"?{type:0,modifier:0,keycode:0}:"",o=t==="labels"?r.labels:r[t];await Ea(e,t,a,o?.[a]??i)}async function vt(){if(!_)return;const{v:e,page:t}=_;_=null;const a=e.profile===s.activeProfileIdx||t===0;if(s.connected&&a)try{await Ri(e,t,!0)}catch{}ce("applied")}async function Oi(){if(!s.connected)return;const e=Li?wl(s.activeProfileIdx,Nt(),Ii):null;e?.id===_?.v.id&&_?.page===Nt()||(_&&await vt(),e&&await Ni(e.id))}async function Ml(){s.connected&&(_&&(_.page!==Nt()||_.v.profile!==s.activeProfileIdx)&&await vt(),await Oi())}async function kl(e){const t=_;t&&await vt();try{return await e()}finally{t&&await Ni(t.v.id)}}function qi(e,t,a,n){const r=le(a,n);return he(t,"keys",r)??e.keys[r]??{modifier:0,keycode:0}}function Ee(e,t,a,n){const r=pe(a,n);return he(t,"rotary",r)??e.rotary?.[r]??{type:0,modifier:0,keycode:0}}function _i(e,t,a,n){const r=V(a,n);return r<0?"":he(t,"labels",r)??e.labels[r]??""}const Yn="orby-backup",zi=4;function la(e){let t="";for(const a of e)t+=a.toString(16).padStart(2,"0");return t}function kr(e){const t=new Uint8Array(e.length/2);for(let a=0;a<t.length;a++)t[a]=parseInt(e.substr(a*2,2),16);return t}async function El(e=()=>{}){if(!s.connected)throw new Error("Teclado no conectado");const t=s.profiles.length||1,a={format:Yn,version:zi,savedAt:new Date().toISOString(),firmware:s.deviceInfo?.fw||null,timeout:s.timeout,activeProfile:s.activeProfileIdx,profiles:[]};for(let n=0;n<t;n++){e(`Leyendo perfil ${n+1} de ${t}…`);const r=await pi(n),i={};for(let c=0;c<20;c++){if(!(r.oledMask&1<<c))continue;e(`Perfil ${n+1}: icono ${c+1} de 20…`);const u=await An(n,c);u&&(i[c]=la(u))}let o=null;if((r.pages[0]?.oledMask||0)&1<<20){e(`Perfil ${n+1}: icono del perfil…`);const c=await Tn(n,0,20);c&&(o=la(c))}a.profiles.push({name:r.name,labels:r.labels,keys:r.keys,rotary:r.rotary,scroll:r.scroll,pageCount:r.pageCount||1,pages:(r.pages||[]).map(c=>({labels:c.labels,keys:c.keys,rotary:c.rotary,scroll:c.scroll,oledMask:c.oledMask})),iconsPage:r.pageIdx||0,icons:i,profileIcon:o})}return a}function Pl(){return{format:Yn,version:zi,savedAt:new Date().toISOString(),firmware:s.deviceInfo?.fw||null,timeout:s.timeout,activeProfile:s.activeProfileIdx,maxProfiles:s.maxProfiles,profiles:s.profiles.map(e=>({name:e.name,labels:e.labels,keys:e.keys,rotary:e.rotary,scroll:e.scroll,pageCount:e.pageCount||1,maxPages:e.maxPages||1,pages:(e.pages||[]).map(t=>({labels:t.labels,keys:t.keys,rotary:t.rotary,scroll:t.scroll,oledMask:t.oledMask})),iconsPage:e.pageIdx||0,icons:Cl(e.idx,e.pageIdx||0),profileIcon:Sl(Hn(e.idx))}))}}function Cl(e,t){const a={};for(let n=0;n<20;n++){const r=ye(e,n,t);r&&(a[n]=la(r))}return a}function Sl(e){return e?la(e):null}async function Il(e,t=()=>{}){if(e?.format!==Yn)throw new Error("El archivo no es una copia de Orby");if(!s.connected)throw new Error("Teclado no conectado");const a=Math.min(s.maxProfiles,e.profiles.length);for(;s.profiles.length<a;)t(`Creando perfil ${s.profiles.length+1}…`),await di(),await fe();for(let n=0;n<a;n++){const r=e.profiles[n];t(`Escribiendo perfil ${n+1} de ${a}…`),r.name&&await ci(n,r.name);const i=Math.min(r.pageCount||1,s.maxPages||1);for(;D(s.profiles[n])<i;)t(`Perfil ${n+1}: creando página ${D(s.profiles[n])+1}…`),await hi(n,!1),await fe();for(let u=0;u<20;u++)await Ht(n,u,r.labels?.[u]??"");for(let u=0;u<24;u++){const p=r.keys?.[u]||{modifier:0,keycode:0};await at(n,u,p.modifier,p.keycode)}for(let u=0;u<16;u++){const p=r.rotary?.[u];p&&await Vt(n,u,p.type,p.modifier,p.keycode)}for(let u=0;u<2;u++){const p=r.scroll?.[u]??e.scroll;p?.detentsPerRev&&await li(n,u,p.detentsPerRev,p.invert)}const o=r.icons||{},c=Object.keys(o);for(let u=0;u<c.length;u++){const p=Number(c[u]);t(`Perfil ${n+1}: icono ${u+1} de ${c.length}…`),await ve(n,p,kr(o[p]))}typeof r.profileIcon=="string"&&r.profileIcon&&(t(`Perfil ${n+1}: icono del perfil…`),await ve(n,20,kr(r.profileIcon)))}Ma(),z(),await fe()}async function Ll(e){try{e("Leyendo el teclado…");const t=await El(e);e("Guardando archivo…");const a=await window.orby.saveBackup(t);if(a.canceled)return;if(!a.ok)throw new Error(a.error||"error desconocido");g("Copia guardada correctamente")}catch(t){g(`No se pudo hacer la copia: ${t.message}`,"error")}finally{e(null)}}async function Al(e){try{const t=await window.orby.loadBackup();if(t.canceled)return;if(!t.ok)throw new Error(t.error||"error desconocido");const a=t.data?.profiles?.length??0;if(!confirm(`Se sobrescribirán los ${a} perfiles del teclado con la copia del ${new Date(t.data.savedAt).toLocaleString("es-ES")}.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`))return;e("Restaurando…"),await Il(t.data,e),g("Copia restaurada; pulsa Guardar en Flash para fijarla")}catch(t){g(`No se pudo restaurar: ${t.message}`,"error")}finally{e(null)}}let Qn=!1,ln=null;const Tl=400,Bi=2;async function Rl(){let e=null;try{e=(await window.orby.getConfig())?.deviceMirror??null}catch{e=null}return Qn=!0,!e?.snapshot?.profiles?.length||s.profiles.length?!1:(Nl(e),console.debug(`[mirror] copia del PC cargada: ${s.profiles.length} perfiles,`,`${Object.keys(e.icons||{}).length} iconos`),te(),!0)}function Nl(e){const t=e.snapshot;s.profiles=t.profiles.map((n,r)=>Ol(n,r)),s.activeProfileIdx=Math.min(t.activeProfile||0,s.profiles.length-1),Number.isInteger(t.timeout)&&(s.timeout=t.timeout),Number.isInteger(t.maxProfiles)&&(s.maxProfiles=t.maxProfiles);const a=s.profiles[s.activeProfileIdx];if(s.pageIdx=0,s.pageCount=a?.pageCount||1,s.maxPages=Math.max(...s.profiles.map(n=>n.maxPages||1),1),Zs(e.icons),(e.iconsVersion||0)<Bi)for(const n of s.profiles)D(n)>1&&Ei(n.idx)}function Ol(e,t){const a={idx:t,name:e.name||"",pageCount:e.pageCount||1,maxPages:e.maxPages||1,pageIdx:0,pages:[]},n=e.pages&&e.pages.length?e.pages:[{labels:e.labels,keys:e.keys,rotary:e.rotary,scroll:e.scroll,oledMask:e.oledMask}];return a.pages=n.map(r=>{const i=Qa();return ea(i.labels,r?.labels,""),ea(i.keys,r?.keys,null),ea(i.rotary,r?.rotary,null),ea(i.scroll,r?.scroll,null),i.oledMask=r?.oledMask||0,i}),a.pageCount=Math.max(1,Math.min(a.pageCount,a.pages.length)),ui(a)}function ea(e,t,a){if(Array.isArray(t))for(let n=0;n<e.length&&n<t.length;n++)t[n]===void 0||t[n]===null||(e[n]=a===""?String(t[n]):t[n])}function ql(){rt(()=>Er()),Vn(()=>Er())}function Er(){!Qn||s.syncing||!s.profiles.length||s.connected&&_l()}function _l(){clearTimeout(ln),ln=setTimeout(na,Tl)}let Pr=null;function na(){if(clearTimeout(ln),!Qn||!s.profiles.length)return;const e=Pl(),t=Qs(),a=JSON.stringify({snapshot:{...e,savedAt:null},icons:t});if(a!==Pr)try{window.orby.setConfig({deviceMirror:{savedAt:new Date().toISOString(),snapshot:e,icons:t,iconsVersion:Bi}}),Pr=a}catch(n){console.error("No se pudo guardar el espejo local:",n)}}let Ve=null,ca=null,da=!1;const zl=7e3;function Bl(){Ve=document.getElementById("app-splash"),ca=document.getElementById("splash-msg"),Ve&&setTimeout(ji,zl)}function jl(){return!!Ve&&!da}function ua(e){ca&&!da&&(ca.textContent=e)}function ji(){da||!Ve||(da=!0,Ve.classList.add("gone"),setTimeout(()=>{Ve?.remove(),Ve=null,ca=null},320))}const cn=new Set,re={status:"idle",version:"",newVersion:null,percent:0,error:null};function za(e){e&&(Object.assign(re,e),cn.forEach(t=>t(re)))}function Di(e){return cn.add(e),()=>cn.delete(e)}async function Dl(){if(!window.orby?.updater){za({status:"dev"});return}window.orby.updater.onState(za),Hl(),za(await window.orby.updater.get())}function Hl(){let e=null;const t=()=>{s.dirty!==e&&(e=s.dirty,window.orby.updater.ocupado(s.dirty))};rt(t),t()}function Vl(){return window.orby?.updater?.check?.()??null}function Hi(){return window.orby?.updater?.install?.()??!1}function Fl(){switch(re.status){case"dev":return"No disponible en modo desarrollo";case"checking":return"Comprobando…";case"downloading":return`Descargando ${re.newVersion} (${re.percent}%)`;case"downloaded":return`${re.newVersion} lista: se instalará al guardar los cambios`;case"error":return`Error: ${re.error}`;default:return"Estás en la última versión"}}let ut=[],Vi=!1;const Fi=new Set;async function ra(){try{ut=await window.orby.plugins.list()}catch{ut=[]}Vi=!0;for(const e of Fi)e();return ut}const Wl=ra;function Zn(e){Fi.add(e),Vi&&e()}function Ul(){return ut}function Jn(){return ut.filter(e=>e.enabled&&!e.error)}function Ze(e){return ut.find(t=>t.id===e)||null}function Pa(e,t){return(e?.actions||[]).filter(a=>a.targets.includes(t))}function Wi(e){return Jn().filter(t=>Pa(t,e).length>0)}function Ca(e,t){const a=Ze(e),n=(a?.actions||[]).find(r=>r.op===t)||null;return{plugin:a,action:n}}function Sa(e){return e?.views||[]}function Gl(){return Jn().filter(e=>e.hasRead&&Sa(e).length>0)}function Kl(e,t){const a=Ze(e),n=Sa(a).find(r=>r.op===t)||null;return{plugin:a,view:n}}async function Xl(e,t){try{return await window.orby.plugins.read(e,t)}catch{return{ok:!1}}}function Ui(e){if(!e)return"Sin asignar";const{plugin:t,action:a}=Ca(e.plugin,e.op);if(!t)return`Complemento «${e.plugin}» (no instalado)`;if(!a){const{view:n}=Kl(e.plugin,e.op);return n?`${t.name}: ${n.label.toLowerCase()} (pantalla)`:`${t.name}: acción desconocida`}if(a.value&&Number.isFinite(e.value))return`${a.label} (${e.value})`;if(a.targets.includes("turn")&&Number(e.value)){const n=Number(e.value)<0?"−":"+";return`${a.label} ${n}${Math.abs(Number(e.value))}`}return`${t.name}: ${a.label.toLowerCase()}`}const Cr="2.0",Mt="4.6",er={read:{since:"2.0",label:"leer la configuración del teclado"},profiles:{since:"3.0",label:"crear y borrar perfiles"},superDial:{since:"3.0",label:"mandos en la capa SUPER"},perProfileWheel:{since:"3.0",label:"rueda ajustable por perfil"},pages:{since:"4.0",label:"páginas dentro de un perfil",flag:"maxpages"},macros:{since:"4.0",label:"secuencias reproducidas por el teclado",flag:"macros"},hash:{since:"4.1",label:"sincronización por huella y precarga de iconos",flag:"hash"},bootsel:{since:"4.2",label:"actualizar el firmware desde la app sin tocar el botón",flag:"bootsel"},profileIcon:{since:"4.4",label:"icono propio para cada perfil",flag:"picon"},hostApp:{since:"4.5",label:"tachar en las pantallas las teclas que necesitan la app",flag:"hostapp"}};function dn(e,t){const a=i=>String(i??"").split(".").map(o=>parseInt(o,10)||0),n=a(e),r=a(t);for(let i=0;i<Math.max(n.length,r.length);i++){const o=(n[i]||0)-(r[i]||0);if(o)return o<0?-1:1}return 0}function Gi(e){return e?.fw||"0.0"}function st(e,t){const a=er[t];if(!a)return!1;if(a.flag&&e&&a.flag in e){const n=e[a.flag];return n!=="0"&&n!==""&&n!=null}return dn(Gi(e),a.since)>=0}function Yl(e){return Object.entries(er).filter(([t])=>!st(e,t)).map(([,t])=>t.label)}function Ki(e){const t=Gi(e);if(dn(t,Cr)<0)return{level:"blocked",fw:t,title:`Firmware ${t}: demasiado antiguo`,detail:`Esta versión de OrbyGUI necesita el firmware ${Cr} o posterior. Flashea el ${Mt} para usar el editor.`};if(dn(t,Mt)>0)return{level:"app-vieja",fw:t,title:`Firmware ${t}: más nuevo que esta app`,detail:`OrbyGUI conoce hasta el ${Mt}. Lo que ya funciona sigue funcionando, pero las funciones nuevas del teclado no se ven hasta que actualices la app.`};const a=Yl(e);return a.length?{level:"recortado",fw:t,title:`Firmware ${t}: faltan funciones`,detail:`Sin flashear el ${Mt} no tendrás: ${a.join(", ")}.`,missing:a}:{level:"ok",fw:t,title:`Firmware ${t}`,detail:"Al día."}}const un=new Set,G={status:"idle",current:null,latest:null,available:!1,percent:0,manual:!1,error:null};function pa(e){e&&(Object.assign(G,e),un.forEach(t=>t(G)))}function Ql(e){return un.add(e),()=>un.delete(e)}function Ia(){return!!window.orby?.firmware}async function Zl(){Ia()&&(window.orby.firmware.onState(pa),pa(await window.orby.firmware.get()))}async function Xi(){if(!Ia())return null;const e=await window.orby.firmware.check({maxFw:Mt,currentFw:s.deviceInfo?.fw??null});return pa(e),e}async function Jl(){if(!Ia())return null;const e=await window.orby.firmware.update({viaBootsel:st(s.deviceInfo,"bootsel")});return pa(e),e}function ec(){return window.orby?.firmware?.cancel?.()??null}function tc(){switch(G.status){case"checking":return"Comprobando…";case"downloading":return`Descargando ${G.latest?.version??""} (${G.percent}%)`;case"bootsel":return G.manual?"Desenchufa el teclado, enchúfalo con BOOTSEL pulsado y suéltalo":"Reiniciando el teclado en modo de actualización…";case"flashing":return"Copiando el firmware… no desconectes el teclado";case"done":return"Firmware actualizado. El teclado se está reiniciando";case"error":return`Error: ${G.error}`;default:return G.latest?G.available?`Disponible la ${G.latest.version}`:"El teclado está al día":"Sin comprobar"}}function Sr(){return["checking","downloading","bootsel","flashing"].includes(G.status)}const La=[{bit:1,label:"Ctrl",short:"Ctrl"},{bit:2,label:"Shift",short:"Shift"},{bit:4,label:"Alt",short:"Alt"},{bit:8,label:"Win",short:"Win"},{bit:16,label:"Ctrl der",short:"RCtrl"},{bit:32,label:"Shift der",short:"RShift"},{bit:64,label:"AltGr",short:"AltGr"},{bit:128,label:"Win der",short:"RWin"}],mt=254,Je=253,et=252,M=251,Xt=[{index:7,label:"Subir volumen"},{index:8,label:"Bajar volumen"},{index:9,label:"Silenciar"},{index:10,label:"Subir brillo"},{index:11,label:"Bajar brillo"},{index:3,label:"Reproducir / Pausa"},{index:4,label:"Detener"},{index:5,label:"Pista siguiente"},{index:6,label:"Pista anterior"},{index:12,label:"Abrir navegador"},{index:1,label:"Abrir explorador"},{index:2,label:"Abrir calculadora"},{index:13,label:"Abrir correo"},{index:14,label:"Buscar"},{index:15,label:"Acercar (zoom +)"},{index:16,label:"Alejar (zoom −)"}],Aa=[{id:"volumen",label:"Volumen",up:7,down:8},{id:"brillo",label:"Brillo",up:10,down:11},{id:"zoom",label:"Zoom",up:15,down:16}],ac=new Set(Aa.flatMap(e=>[e.up,e.down])),nc=[...Aa.map(e=>({pairId:e.id,label:e.label,up:e.up,down:e.down})),...Xt.filter(e=>!ac.has(e.index))];function rc(e){return Aa.find(t=>t.up===e||t.down===e)||null}const S={NONE:0,CONSUMER:1,KEY:2,SCROLL_V:3,SCROLL_H:4,ZOOM:5},R={ENC1_CW:0,ENC1_CCW:1,ENC1_CLICK:2,ENC2_CW:3,ENC2_CCW:4,ENC2_CLICK:5,WHEEL_CW:6,WHEEL_CCW:7};function tr(e){return e===S.SCROLL_V||e===S.SCROLL_H||e===S.ZOOM}function ic(e){if(!e)return"Sin asignar";switch(e.type){case S.SCROLL_V:return"Desplazar vertical";case S.SCROLL_H:return"Desplazar horizontal";case S.ZOOM:return"Zoom (Ctrl + rueda)";case S.CONSUMER:{const t=Xt.find(a=>a.index===e.keycode);return t?t.label:`Multimedia ${e.keycode}`}case S.KEY:return Ot(e.modifier,e.keycode);default:return"Sin asignar"}}function xt(e,t,a){const n=[];for(let r=e;r<=t;r++)n.push(a(r));return n}const Yi=[{name:"Letras",keys:xt(0,25,e=>({code:4+e,label:String.fromCharCode(65+e)}))},{name:"Números",keys:[...xt(0,8,e=>({code:30+e,label:String(e+1)})),{code:39,label:"0"}]},{name:"Función",keys:[...xt(0,11,e=>({code:58+e,label:`F${e+1}`})),...xt(0,11,e=>({code:104+e,label:`F${e+13}`}))]},{name:"Edición",keys:[{code:40,label:"Enter"},{code:41,label:"Esc"},{code:42,label:"Retroceso"},{code:43,label:"Tab"},{code:44,label:"Espacio"},{code:73,label:"Insert"},{code:76,label:"Supr"},{code:74,label:"Inicio"},{code:77,label:"Fin"},{code:75,label:"Re Pág"},{code:78,label:"Av Pág"},{code:70,label:"Impr Pant"}]},{name:"Flechas",keys:[{code:80,label:"←"},{code:79,label:"→"},{code:82,label:"↑"},{code:81,label:"↓"}]},{name:"Símbolos",keys:[{code:45,label:"-"},{code:46,label:"="},{code:47,label:"["},{code:48,label:"]"},{code:49,label:"\\"},{code:51,label:";"},{code:52,label:"'"},{code:53,label:"`"},{code:54,label:","},{code:55,label:"."},{code:56,label:"/"}]},{name:"Teclado numérico",keys:[{code:84,label:"Num /"},{code:85,label:"Num *"},{code:86,label:"Num -"},{code:87,label:"Num +"},{code:88,label:"Num Enter"},...xt(1,9,e=>({code:88+e,label:`Num ${e}`})),{code:98,label:"Num 0"}]}],Qi=new Map;for(const e of Yi)for(const t of e.keys)Qi.set(t.code,t.label);function oc(e){return Qi.get(e)||(e?`0x${e.toString(16).toUpperCase()}`:"—")}function Ot(e,t){if(e===mt){const n=Xt.find(r=>r.index===t);return n?n.label:`Multimedia ${t}`}if(e===Je)return`Ir a la página ${t}`;if(e===et)return"Estado de páginas";if(e===M)return`Secuencia ${t}`;if(!e&&!t)return"Sin asignar";const a=La.filter(n=>e&n.bit).map(n=>n.short);return t&&a.push(oc(t)),a.join(" + ")}const sc=(()=>{const e=new Map;for(let a=0;a<26;a++)e.set(`Key${String.fromCharCode(65+a)}`,4+a);for(let a=1;a<=9;a++)e.set(`Digit${a}`,30+a-1);e.set("Digit0",39);for(let a=1;a<=24;a++)e.set(`F${a}`,a<=12?58+a-1:104+a-13);const t={Enter:40,Escape:41,Backspace:42,Tab:43,Space:44,Minus:45,Equal:46,BracketLeft:47,BracketRight:48,Backslash:49,Semicolon:51,Quote:52,Backquote:53,Comma:54,Period:55,Slash:56,CapsLock:57,PrintScreen:70,ScrollLock:71,Pause:72,Insert:73,Home:74,PageUp:75,Delete:76,End:77,PageDown:78,ArrowRight:79,ArrowLeft:80,ArrowDown:81,ArrowUp:82,NumLock:83,NumpadDivide:84,NumpadMultiply:85,NumpadSubtract:86,NumpadAdd:87,NumpadEnter:88,Numpad0:98};for(const[a,n]of Object.entries(t))e.set(a,n);for(let a=1;a<=9;a++)e.set(`Numpad${a}`,88+a);return e})();function Ir(e){const t=sc.get(e.code)||0;let a=0;return e.ctrlKey&&(a|=1),e.shiftKey&&(a|=2),e.altKey&&(a|=4),e.metaKey&&(a|=8),{modifier:a,keycode:t}}let ge=[];function lc(){return ge}function cc(e){ge=ge.filter(t=>t.id!==e),P(e)}async function gt(){try{ge=(await window.orby.getConfig())?.macros||[]}catch{ge=[]}}function P(e){window.orby.setConfig({macros:ge}),e!==void 0&&ao(e)}function L(e){return ge.find(t=>t.id===e)}function oe(e){let t=L(e);return t||(t={id:e,actions:[]},ge.push(t),P(e)),t}function me(){const e=new Set(ge.map(a=>a.id));let t=1;for(;e.has(t);)t++;return t}const Zi={delay:1,hotkey:2,mouse_move:3,mouse_click:4},Ji=48,dc={left:0,middle:1,right:2},uc=64;function ar(){const e=parseInt(s.deviceInfo?.maxmacros??"",10);return Number.isFinite(e)&&e>0?e:uc}const eo={sleep:"Suspender",hibernate:"Hibernar",restart:"Reiniciar",shutdown:"Apagar",lock:"Bloquear pantalla",logoff:"Cerrar sesión"},to=20;function qt(e){const t=e?.actions||[];return t.length>0&&t.length<=Ji&&e.id<ar()&&t.every(a=>a.type in Zi)}function pc(e){return e.actions.map(t=>{const a=Zi[t.type];let n=0,r=0;t.type==="delay"?n=Math.max(0,Math.min(32767,Math.round(t.ms)||0)):t.type==="hotkey"?(n=t.modifier||0,r=t.keycode||0):t.type==="mouse_move"?(n=t.dx||0,r=t.dy||0):t.type==="mouse_click"&&(n=dc[t.button]??0);const o=t.type==="hotkey"||t.type==="mouse_click"?Math.max(1,Math.min(255,t.count||1)):1,c=o>1?Math.max(0,Math.min(32767,Math.round(t.gap??to))):0;return{type:a,a:n,b:r,repeat:o,gap:c}})}async function Lr(e,t){let a;try{a=await ms(e)}catch{return!1}return!a||a.length!==t.length?!1:t.every((n,r)=>{const i=a[r];return i&&i.type===n.type&&i.a===n.a&&i.b===n.b&&i.repeat===n.repeat&&i.gap===n.gap})}async function ao(e){if(!s.connected||!st(s.deviceInfo,"macros")||e>=ar())return;const t=L(e);if(!t||!qt(t)){if(await Lr(e,[]))return;try{await hs(e)}catch{}z();return}const a=pc(t);if(!await Lr(e,a))try{for(let n=0;n<a.length;n++){const r=a[n];await ps(e,n,r.type,r.a,r.b,r.repeat,r.gap)}await fs(e,a.length),z()}catch{}}async function no(){if(!st(s.deviceInfo,"macros"))return;await gt();const e=await fc();for(const n of ge)!qt(n)&&e&&!e.includes(n.id)||await ao(n.id);const t=ar(),a=ge.filter(n=>qt(n)&&n.id<t).map(n=>n.id);window.orby.setConfig({macrosOnDevice:a})}async function fc(){try{const e=await window.orby.getConfig();return Array.isArray(e?.macrosOnDevice)?e.macrosOnDevice:null}catch{return null}}function fa(e){const t=L(e)?.actions||[];return t.length===1&&t[0].type==="open_app"}function ha(e){const t=L(e)?.actions||[];return t.length===1&&t[0].type==="text"}function Yt(e){const t=L(e)?.actions||[];return t.length===1&&t[0].type==="system_power"}function $t(e){const t=L(e)?.actions||[];return t.length===1&&t[0].type==="plugin"}function Be(e){return $t(e)?L(e).actions[0]:null}function _t(e){return e?.type===S.KEY&&e.modifier===M&&$t(e.keycode)?Ui(L(e.keycode).actions[0]):ic(e)}function ro(e){return L(e)?.kind==="recording"}function nr(e){return L(e)?.kind==="recording-reset"}function It(e){return ro(e)||nr(e)}function hc(e){if(e.modifier!==M)return Ot(e.modifier,e.keycode);if(nr(e.keycode))return"Borrar grabación";if(ro(e.keycode))return L(e.keycode).events?.length?"Reproducir grabación":"Grabar operación";if(fa(e.keycode)){const t=L(e.keycode).actions[0].target||"";return t?`Abrir ${t.split(/[\\/]/).pop()}`:"Abrir…"}if(ha(e.keycode)){const t=(L(e.keycode).actions[0].text||"").replace(/\s+/g," ").trim();return t?`Escribir "${t.length>14?`${t.slice(0,14)}…`:t}"`:"Escribir texto"}return Yt(e.keycode)?eo[L(e.keycode).actions[0].mode]||"Energía":$t(e.keycode)?Ui(L(e.keycode).actions[0]):Ot(e.modifier,e.keycode)}function io(e){if(!e||e.modifier!==M)return!1;const t=L(e.keycode);return t?!qt(t):!1}function pn(e){return!e||e.type!==S.KEY?!1:io(e)}const mc=2e3,Ar=new Map;function gc(e){let t=Ar.get(e);if(!t){const a=f(e,96).replace("currentColor","#fff");t=ki(a,{size:96}),Ar.set(e,t)}return t}function yc(e,t,a,n){const r=it(e),i=n/Math.max(1,r.height);return{x:t-r.x*i,y:a-r.y*i,scale:i,threshold:128,blur:0,dither:!1,invert:"none"}}async function Tr(e,t,a,n,r){try{const i=await gc(t),o=Gt(i,yc(i,a,n,r));return Dn(e,o,"merge")}catch(i){return console.error(`[live-oled] icono "${t}":`,i),e}}const Ba=4,ja=w-4,Da=26,Ha=34;function bc(e,t){for(let o=Ba;o<=ja;o++)ne(e,o,Da,1),ne(e,o,Ha,1);for(let o=Da;o<=Ha;o++)ne(e,Ba,o,1),ne(e,ja,o,1);const a=Math.max(0,Math.min(100,Number(t)||0))/100,n=Ba+2,r=ja-2,i=Math.round(n+(r-n)*a);for(let o=n;o<=i;o++)for(let c=Da+2;c<=Ha-2;c++)ne(e,o,c,1)}async function vc(e,t,a){let n=Ut();return n=await Tr(n,e,4,3,15),n=await Tr(n,t,w-19,3,15),bc(n,a),n}function $c(){if(!s.connected)return[];const e=s.activeProfileIdx,a=s.profiles[e]?.pages?.[s.pageIdx];if(!a)return[];const n=s.superActive?"super":"normal",r=[];for(let i=0;i<_e.length;i++){if(!_e[i])continue;const o=a.keys[le(i,n)];if(!o||o.modifier!==M)continue;const c=L(o.keycode),u=c?.actions?.length===1&&c.actions[0].type==="plugin"?c.actions[0]:null;if(!u)continue;const p=Ze(u.plugin);if(!p?.enabled||!p.hasRead)continue;const m=Sa(p).find(v=>v.op===u.op);m&&r.push({key:`${e}:${s.pageIdx}:${V(i,n)}`,profileIdx:e,pageIdx:s.pageIdx,oledSlot:V(i,n),pluginId:p.id,pluginIcon:p.icon,op:u.op,viewIcon:m.icon})}return r}let ma=null;const Lt=new Map;async function xc(e){if(!(e.profileIdx!==s.activeProfileIdx||e.pageIdx!==s.pageIdx))try{const t=ye(e.profileIdx,e.oledSlot,e.pageIdx);t?await ve(e.profileIdx,e.oledSlot,t):await Ft(e.profileIdx,e.oledSlot)}catch{}}async function wc(e){const t=await Xl(e.pluginId,e.op);if(!t.ok){console.warn(`[live-oled] "${e.op}" de "${e.pluginId}" no respondió`);return}try{const a=await vc(e.pluginIcon,e.viewIcon,t.value);await ve(e.profileIdx,e.oledSlot,a)}catch(a){console.error(`[live-oled] no se pudo subir la tecla ${e.oledSlot}:`,a)}}let Rr="";async function fn(){const e=$c(),t=new Set(e.map(n=>n.key)),a=e.map(n=>n.key).join(",");a!==Rr&&(Rr=a,console.debug("[live-oled] teclas visoras activas:",e));for(const[n,r]of Lt)t.has(n)||(Lt.delete(n),xc(r));for(const n of e)Lt.set(n.key,n);await Promise.all(e.map(wc))}function Nr(){ma||(ma=setInterval(fn,mc),fn())}function Or(){clearInterval(ma),ma=null}function Mc(){H("plugins")&&(gt(),rt(()=>s.connected?Nr():(Or(),Lt.clear())),Zn(()=>{s.connected&&fn()}),W("connected",()=>{gt(),Nr()}),W("disconnected",()=>{Or(),Lt.clear()}))}let qr=null,Va=0;function oo(){const e=document.getElementById("keyboard-visualizer");let t=`
    <div class="orby-board">
      <div class="board-keys">`;for(let a=1;a<=12;a++){const n=_e[a-1]!==0;t+=`
      <div class="hw-key ${n?"":"no-screen"}" id="hw-key-${a}">
        ${n?`<div class="oled-screen" id="hw-oled-${a}">--</div>`:`<span class="hw-key-role">${a===ft?"SUPER":"MENU"}</span>`}
      </div>`}t+=`
      </div>
      <div class="board-controls">
        <div class="encoder-row">
          <div class="hw-encoder" id="hw-enc-1"><span class="hw-tag">ENC 1</span></div>
          <div class="hw-encoder" id="hw-enc-2"><span class="hw-tag">ENC 2</span></div>
        </div>
        <div class="hw-wheel" id="hw-scroll">
          <div class="hw-wheel-face">
            ${Kn("hw-wheel-needle")}
            <div class="hw-wheel-hub"></div>
          </div>
          <span class="hw-tag">RUEDA</span>
        </div>
      </div>
    </div>`,e.innerHTML=t,W("telemetry",kc),Ci(so),Vn(Ge),Ge()}function so(e){const t=document.getElementById("hw-wheel-needle");t&&(t.style.transform=`rotate(${e}deg)`)}function Ta(){const e=document.querySelector(".hw-wheel-face");e&&(e.querySelector(".dial-marker")?.remove(),e.insertAdjacentHTML("afterbegin",Kn("hw-wheel-needle")),so(Un()))}function kc(e){if(e.startsWith("KEY_EV:")){const[,t,a]=e.split(":"),n=parseInt(t,10),r=a==="1";n===ft&&(s.superActive=r,te(),Ge()),r?(Fa(`hw-key-${n}`),kt(`Tecla ${n}`,Ec(n))):document.getElementById(`hw-key-${n}`)?.classList.remove("active")}else if(e.startsWith("ENC:")){const[,t,a]=e.split(":");Fa(`hw-enc-${t}`);const n=parseInt(a,10);kt(`Encoder ${t}`,`${n>0?"+":""}${n}`)}else if(e.startsWith("ENC_SW:")){const[,t,a]=e.split(":");a==="1"&&(Fa(`hw-enc-${t}`),kt(`Encoder ${t}`,"Click"))}else e.startsWith("WHEEL:")?Pc(parseInt(e.slice(6),10)):e.startsWith("MODE:")?(s.deviceMode=e.split(":")[1],te(),Ge(),kt("Modo",s.deviceMode)):e.startsWith("PROFILE:OK:")&&(s.activeProfileIdx=parseInt(e.split(":")[2],10),te(),Ge())}function Ec(e){const t=bt();if(!t)return"";const a=s.superActive?"super":"normal";return yi(t,e-1,a)||""}function Pc(e){if(!Number.isFinite(e))return;Va+=e;const t=document.getElementById("hw-scroll");if(!t)return;t.classList.add("active");const a=$i().detentsPerRev||60,n=Va*a/4096;kt("Scroll",`${n>=0?"+":""}${n.toFixed(2)} clics`),clearTimeout(qr),qr=setTimeout(()=>{t.classList.remove("active"),Va=0},600)}function Fa(e){const t=document.getElementById(e);t&&(t.classList.add("active"),setTimeout(()=>t.classList.remove("active"),160))}function kt(e,t){const a=document.getElementById("live-event-feed");a&&(a.innerHTML=`
    <div class="big-event">
      <span class="big-event-title">${e}</span>
      ${t?`<span class="big-event-detail">${t}</span>`:""}
    </div>`)}function Ge(){const e=bt(),t=s.superActive?"super":"normal";document.getElementById("lbl-active-profile").textContent=e?e.name:"--",document.getElementById("lbl-active-mode").textContent=s.deviceMode;const a=document.getElementById("row-active-page");if(a){const i=e?D(e):1;a.hidden=!nt(),document.getElementById("lbl-active-page").textContent=`${Math.min(s.pageIdx,i-1)+1} de ${i}`}const n=document.getElementById("lbl-super-state");n.textContent=s.superActive?"Activa":"Inactiva",n.classList.toggle("is-on",s.superActive);const r=document.getElementById("lbl-scroll-mode");if(r){const i=$i();r.textContent=`${i.detentsPerRev} clics/vuelta · ${i.hires?"suave":"clásico"}`,r.classList.toggle("is-on",i.hires)}for(let i=1;i<=12;i++){const o=document.getElementById(`hw-oled-${i}`);if(!o)continue;const c=V(i-1,t);(e&&c>=0?ye(s.activeProfileIdx,c):null)?o.innerHTML=`<canvas class="okey-canvas" data-bmp="${ue(s.activeProfileIdx,c)}"></canvas>`:o.textContent=e&&yi(e,i-1,t)||"--"}Kt(document.getElementById("keyboard-visualizer"))}const Cc=Object.freeze(Object.defineProperty({__proto__:null,init:oo,refreshMarker:Ta,render:Ge},Symbol.toStringTag,{value:"Module"})),lo=[{name:"Izquierdo",icon:"reset",parts:[{slot:R.ENC1_CW,label:"Giro horario",short:"↻"},{slot:R.ENC1_CCW,label:"Giro antihorario",short:"↺"},{slot:R.ENC1_CLICK,label:"Pulsación",short:"⏺",discrete:!0}]},{name:"Derecho",icon:"reset",parts:[{slot:R.ENC2_CW,label:"Giro horario",short:"↻"},{slot:R.ENC2_CCW,label:"Giro antihorario",short:"↺"},{slot:R.ENC2_CLICK,label:"Pulsación",short:"⏺",discrete:!0}]}],Sc={name:"Rueda de scroll",icon:"wheel",parts:[{slot:R.WHEEL_CW,label:"Hacia abajo",short:"↓"},{slot:R.WHEEL_CCW,label:"Hacia arriba",short:"↑"}]},Ic=[{type:S.NONE,label:"Nada"},{type:S.CONSUMER,label:"Multimedia / sistema"},{type:S.KEY,label:"Atajo de teclado"},{type:S.SCROLL_V,label:"Desplazar vertical",turnOnly:!0},{type:S.SCROLL_H,label:"Desplazar horizontal",turnOnly:!0},{type:S.ZOOM,label:"Zoom (Ctrl + rueda)",turnOnly:!0}],Lc=[{value:12,name:"Preciso",desc:"Timeline, PCB, edición fina"},{value:30,name:"Suave",desc:"Lectura y navegación lenta"},{value:60,name:"Estándar",desc:"Equivalente a rueda de ratón"},{value:120,name:"Rápido",desc:"Documentos y logs largos"}],co=50,Ac=360,zt=new Set([R.ENC1_CCW,R.ENC2_CCW,R.WHEEL_CW]),Qt={[R.ENC1_CW]:R.ENC1_CCW,[R.ENC1_CCW]:R.ENC1_CW,[R.ENC2_CW]:R.ENC2_CCW,[R.ENC2_CCW]:R.ENC2_CW,[R.WHEEL_CW]:R.WHEEL_CCW,[R.WHEEL_CCW]:R.WHEEL_CW};let ga={render:()=>{},renderKeyGrid:()=>{}};function Tc(e){ga={...ga,...e}}function T(){ga.render()}function _r(){ga.renderKeyGrid()}const d={editingProfile:0,layer:"normal",variantId:null,selected:null,tab:"shortcut",capturing:!1,busy:!1};function ya(){const e=s.profiles[d.editingProfile];return e&&e.pageIdx||0}function U(){const e=d.variantId?ot(d.variantId):null;return e&&e.profile===d.editingProfile&&e.page===ya()?e:null}function Te(){return d.selected?.kind==="key"?d.selected.index:null}function xe(){return d.selected?.kind==="rotary"?d.selected.slot:null}function uo(e){for(const t of[...lo,Sc]){const a=t.parts.find(n=>n.slot===e);if(a)return{group:t,part:a}}return null}function N(){return s.profiles[d.editingProfile]||null}function Re(){return document.getElementById("view-profiles")?.classList.contains("active")}function J(){const e=N(),t=Te();return!e||t===null?{modifier:0,keycode:0}:qi(e,U(),t,d.layer)}function qe(){const e=N(),t=xe();return!e||t===null?{type:0,modifier:0,keycode:0}:Ee(e,U(),t,d.layer)}function rr(){return Rn(N(),d.layer)}function A(){const e=d.selected?.kind==="rotary"?qe():J();return e.modifier===M?e.keycode:null}const ee={open:!1,plugin:null,op:null,mode:null,value:0};async function po(e,t){if(!Y())return;const a=N();if(!a)return;const n=U();if(n){if(Xn(n.id,"labels",e,t),_r(),Ae(n.id))try{await Ea(n.id,"labels",e,t)}catch{g("El teclado no confirmó la etiqueta","error")}return}a.labels[e]=t;try{await Ht(d.editingProfile,e,t),z(),await ka("labels",e),_r()}catch{g("El teclado no confirmó la etiqueta","error")}}async function K(e,t){if(!Y())return;const a=N(),n=Te();if(!a||n===null)return;const r=le(n,d.layer),i=U();if(i){if(Xn(i.id,"keys",r,{modifier:e,keycode:t}),T(),Ae(i.id))try{await Ea(i.id,"keys",r,{modifier:e,keycode:t})}catch{g("No se pudo escribir la asignación","error")}return}a.keys[r]={modifier:e,keycode:t},T();try{await at(d.editingProfile,r,e,t),z(),await ka("keys",r)}catch{g("No se pudo escribir la asignación","error")}}async function Rc(){if(!Y())return;const e=N(),t=Te();if(!e||t===null)return;const a=V(t,d.layer);if(!(a<0)){await po(a,"");try{const n=new Uint8Array(Ac);await ve(d.editingProfile,a,n),ze(d.editingProfile,a,n),e.oledMask|=1<<a,z(),T()}catch{g("No se pudo apagar la pantalla de la tecla","error")}}}let de=null;async function Nc(){const e=N(),t=Te();if(!e||t===null)return;const a=J(),n=V(t,d.layer),r=n>=0?_i(e,U(),t,d.layer):"",i=n>=0?ye(d.editingProfile,n):null;let o=null;if(a.modifier===M){const c=L(a.keycode);c&&(o=JSON.parse(JSON.stringify(c.actions||[])))}de={action:{modifier:a.modifier,keycode:a.keycode},label:r,icon:i?Uint8Array.from(i):null,macroActions:o},g("Tecla copiada"),T()}async function Oc(){if(!de||!Y())return;const e=N(),t=Te();if(!e||t===null)return;let{modifier:a,keycode:n}=de.action;if(a===M&&de.macroActions){const i=me(),o=oe(i);o.actions=JSON.parse(JSON.stringify(de.macroActions)),P(i),n=i}await K(a,n);const r=V(t,d.layer);if(r<0){g("Tecla pegada");return}await po(r,de.label||"");try{de.icon?(await ve(d.editingProfile,r,de.icon),ze(d.editingProfile,r,Uint8Array.from(de.icon)),e.oledMask|=1<<r,z()):ye(d.editingProfile,r)&&(await Ft(d.editingProfile,r),ze(d.editingProfile,r,null),e.oledMask&=~(1<<r),z()),T()}catch{g("No se pudo pegar el icono","error")}g("Tecla pegada")}async function fo(e){const t=N();if(!t)return;const a=U();if(a){for(const[n,r]of e)Xn(a.id,"rotary",n,r);if(T(),Ae(a.id))try{for(const[n,r]of e)await Ea(a.id,"rotary",n,r)}catch{g("No se pudo escribir la acción del mando","error")}return}for(const[n,r]of e)t.rotary[n]=r;T();try{for(const[n,r]of e)await Vt(d.editingProfile,n,r.type,r.modifier,r.keycode),await ka("rotary",n);z()}catch{g("No se pudo escribir la acción del mando","error")}}async function Ke(e){if(!Y())return;const t=N(),a=xe();if(!t||a===null)return;const n=pe(a,d.layer),r=Qt[a],i=r!==void 0&&tr(e.type),o=[[n,e]];i&&o.push([pe(r,d.layer),{...e}]),await fo(o)}async function ir(e,t,a,n){Y()&&N()&&await fo([[pe(e,d.layer),t],[pe(a,d.layer),n]])}const zr=[{id:"once",label:"Una vez",desc:"Reproduce la operación de principio a fin y para."},{id:"loop",label:"En bucle",desc:"Repite sin parar hasta que vuelvas a pulsar la tecla."},{id:"hold",label:"Mientras se pulsa",desc:"Repite mientras mantengas la tecla, y para al soltarla."}],qc=[1,2,3,5],ho=3;let pt={id:null,phase:"idle"},yt=null,or=null;const mo="RECORD",go="PLAY",yo="RESET";function lt(e){const t=e===null?null:L(e);return t?.kind==="recording"?t:t?.kind==="recording-reset"&&L(t.target)||null}function _c(e){const t=Zt(e);if(!t)return!1;const a=t.layer==="normal"?"super":"normal",n=s.profiles[t.profile]?.keys[le(t.key,a)];return n?.modifier===M&&L(n.keycode)?.target===e}function zc(e){const t=e?.events||[];return t.length?t[t.length-1].t:0}function Zt(e){for(let t=0;t<s.profiles.length;t++){const a=s.profiles[t];for(const n of["normal","super"])for(let r=0;r<12;r++){const i=a.keys[le(r,n)];if(i?.modifier===M&&i.keycode===e&&V(r,n)>=0)return{profile:t,key:r,layer:n}}}return null}function Bc(e){const t=Zt(e);return t?{profile:t.profile,slot:V(t.key,t.layer)}:null}async function Br(e,t,a){if(t<0)return;const n=vo(a);await sr(async()=>{try{await ve(e,t,n),ze(e,t,Uint8Array.from(n));const r=s.profiles[e];r&&(r.oledMask|=1<<t),z()}catch{g("No se pudo escribir el icono de la tecla","error")}})}async function Bt(e){const t=L(e),a=Zt(e);if(!t||!a)return;await Br(a.profile,V(a.key,a.layer),t.events?.length?go:mo);const n=a.layer==="normal"?"super":"normal",i=s.profiles[a.profile]?.keys[le(a.key,n)];i?.modifier===M&&L(i.keycode)?.target===e&&await Br(a.profile,V(a.key,n),yo)}async function jc(e){const t=N(),a=Te();if(!t||a===null||d.layer!=="normal")return;const n=le(a,"super"),r=me(),i=oe(r);i.kind="recording-reset",i.target=e,i.actions=[];const o=L(e);o&&(o.resetId=r),P(r),t.keys[n]={modifier:M,keycode:r};try{await at(d.editingProfile,n,M,r),z(),await ka("keys",n)}catch{g("No se pudo asignar la tecla de borrado en SUPER","error")}}async function bo(e){const t=L(e),a=Zt(e);if(!t||!a)return;const n=a.layer==="normal"?"super":"normal",r=le(a.key,n),i=s.profiles[a.profile],o=i?.keys[r];if(!(o?.modifier!==M||L(o.keycode)?.target!==e)){cc(o.keycode),i.keys[r]={modifier:0,keycode:0};try{await at(a.profile,r,0,0),await Ft(a.profile,V(a.key,n)),ze(a.profile,V(a.key,n),null),i.oledMask&=~(1<<V(a.key,n)),z()}catch{g("No se pudo quitar la tecla de borrado de SUPER","error")}}}function vo(e){const t=qn(e,{fontSize:20,bold:!0,font:"Segoe UI Black"}),a=jn(t),n=Bn(t),r=Gt(t,{x:Math.round((w-n.width*a)/2),y:Math.round((I-n.height*a)/2),scale:a,threshold:128,blur:0,dither:!1,invert:"none"});return Mi(r),r}let jr=Promise.resolve();function sr(e){const t=jr.then(e,e);return jr=t.catch(()=>{}),t}async function Wa(e,t){if(clearTimeout(or),!s.connected)return;const a=Bc(e);if(!a)return;if(!yt){const r=ye(a.profile,a.slot);yt={...a,bytes:r?Uint8Array.from(r):null}}const n=vo(t);await sr(async()=>{try{await ve(a.profile,a.slot,n)}catch{}})}async function Dc(){clearTimeout(or);const e=yt;yt=null,!(!e||!s.connected)&&await sr(async()=>{try{e.bytes?await ve(e.profile,e.slot,e.bytes):await Ft(e.profile,e.slot)}catch{}})}async function Hc({id:e,phase:t}){pt={id:e,phase:t},t==="recording"?await Wa(e,"REC"):t==="playing"?await Wa(e,"RUN"):t==="saved"||t==="empty"?(await gt(),await Wa(e,t==="saved"?"OK":"VACIO"),or=setTimeout(async()=>{pt={id:null,phase:"idle"},yt=null,await Bt(e),Re()&&T()},1200)):t==="reset"?(await gt(),pt={id:null,phase:"idle"},yt=null,await Bt(e),g("Grabación borrada: la tecla vuelve a estar lista para grabar")):await Dc(),Re()&&T()}function Vc(e){const t=lt(A());t&&(t.mode=e,P(t.id),T())}function Fc(e){const t=lt(A());t&&(t.speed=Number(e)||ho,P(t.id),T())}async function Wc(){const e=lt(A());!e||!e.events?.length||confirm(`Se borrará la operación grabada en esta tecla.

¿Continuar?`)&&(e.events=[],P(e.id),await Bt(e.id),T())}async function Uc(){const e=lt(A());if(e){if(pt.phase!=="recording"&&e.events?.length){if(!confirm(`Se sustituirá la operación grabada en esta tecla.

¿Grabar de nuevo?`))return;e.events=[],await window.orby.setConfig({macros:lc()}),await Bt(e.id),T()}window.orby.recorder.toggle(e.id)}}function Gc(e){const t=lt(e);if(!t)return'<p class="setting-desc">Preparando la grabación de esta tecla…</p>';if(nr(e)){const p=Zt(t.id),m=t.events||[];return`
      <div class="field">
        <span class="field-label">Borrar la grabación</span>
        <div class="rec-state ${m.length?"ok":"off"}">
          ${p?`Esta tecla borra lo grabado en la tecla ${p.key+1} de la capa normal.`:"Tecla de borrado."}
          ${m.length?` Ahora mismo hay ${m.length} eventos guardados.`:" Ahora mismo no hay nada que borrar."}
        </div>
      </div>
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-clear" ${m.length?"":"disabled"}>
          ${f("trash",16)} Borrar ahora
        </button>
      </div>
      <p class="setting-desc">
        Se creó sola al convertir la otra tecla en grabación. Para quitarla, cambia esa tecla a
        cualquier otra pestaña que no sea "Grabar".
      </p>`}const a=t.events||[],n=pt.id===t.id?pt.phase:"idle",r=n==="recording",i=n==="playing",o=t.mode||"once",c=t.speed||ho,u=(zc(t)/1e3).toFixed(1);return`
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
      La pantalla de esta tecla pone <strong>${a.length?go:mo}</strong>
      ${_c(t.id)?`, y con SUPER pone <strong>${yo}</strong>: <strong>SUPER + esta tecla</strong>
           borra lo grabado y la deja lista para grabar otra vez.`:`. La tecla de borrado automática solo se monta al crear la grabación en la capa NORMAL:
           en SUPER no queda otra capa donde ponerla.`}
    </div>

    <div class="field mt-4">
      <span class="field-label">Al reproducir</span>
      <div class="chip-row">
        ${zr.map(p=>`
          <button class="chip ${o===p.id?"on":""}" data-act="rec-mode" data-mode="${p.id}">${p.label}</button>`).join("")}
      </div>
      <p class="setting-desc">${zr.find(p=>p.id===o)?.desc||""}</p>
    </div>

    <div class="field mt-4">
      <span class="field-label">Velocidad de reproducción</span>
      <div class="chip-row">
        ${qc.map(p=>`
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
    </p>`}async function Kc(){const e=me(),t=oe(e);t.kind="recording",t.mode="once",t.events=[],t.actions=[],P(e),await K(M,e),await jc(e),await Bt(e),T()}async function Ra(){let e=await window.orby.foreground.current();if(e)return e;await window.orby.foreground.start();for(let t=0;t<8&&!e;t++)await new Promise(a=>setTimeout(a,300)),e=await window.orby.foreground.current();return e}function lr(e){return Yi.map(t=>`
    <optgroup label="${t.name}">
      ${t.keys.map(a=>`<option value="${a.code}" ${e===a.code?"selected":""}>${x(a.label)}</option>`).join("")}
    </optgroup>`).join("")}function x(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}function Xc(e,t,a,n){const{action:r}=Ca(e,t);r?.value&&(ee.open=!0,ee.plugin=e,ee.op=t,ee.mode=a,ee.value=Number.isFinite(n)?n:r.value.default,T())}function Yc(e){ee.value=e}function Qc(){ee.open=!1,T()}function Zc(){const{plugin:e,op:t,mode:a,value:n}=ee;ee.open=!1,a==="rotary"?hn(e,t,n):xo(e,t,n)}function Jc(e,t){return e.type!==t.type?!1:e.type==="hotkey"?e.modifier===t.modifier&&e.keycode===t.keycode:e.type==="mouse_click"?e.button===t.button:!1}function Ne(e){const t=A();if(t===null)return;const a=oe(t),n=a.actions[a.actions.length-1];if(n&&n.type!=="delay"&&Jc(n,e)){n.count=(n.count||1)+(e.count||1),P(t),T();return}a.actions.length&&a.actions[a.actions.length-1].type!=="delay"&&a.actions.push({type:"delay",ms:co}),a.actions.push({...e,count:e.count||1}),P(t),T()}function Na(){const e=A(),t=e===null?null:L(e);return t?(t.actions.length||t.actions.push({type:"open_app",target:""}),t.actions[0]):null}function ed(e){const t=Na();t&&(t.kind=e,P(A()),T())}function $o(){const e=A(),t=e===null?null:L(e);return t?(t.actions.length||t.actions.push({type:"text",text:""}),t.actions[0]):null}function td(e){const t=J(),a=t.modifier===M&&Yt(t.keycode)?t.keycode:me(),n=oe(a);n.actions=[{type:"system_power",mode:e}],P(a),K(M,a)}function xo(e,t,a){const n=J(),i=Be(n.modifier===M?n.keycode:-1)?.plugin===e?n.keycode:me(),o=oe(i),c={type:"plugin",plugin:e,op:t};Number.isFinite(a)&&(c.value=a),o.actions=[c],P(i),K(M,i)}function ad(e,t){const a=zt.has(e)?-1:1;return Math.sign(Number(t?.value)||0)===-a}function nd(){const e=N(),t=xe(),a=t===null?void 0:Qt[t];if(!e||t===null||a===void 0)return;const n=U(),r=Ee(e,n,t,d.layer),i=Ee(e,n,a,d.layer);ir(t,{...i},a,{...r})}function hn(e,t,a){const{action:n}=Ca(e,t);if(!n)return;const r=xe();if(n.targets.includes("turn")&&n.step>0&&r!==null&&Qt[r]!==void 0){rd(r,e,t,n);return}const o=qe(),u=(o.type===S.KEY&&o.modifier===M?Be(o.keycode):null)?.plugin===e?o.keycode:me(),p=oe(u),m={type:"plugin",plugin:e,op:t};Number.isFinite(a)&&(m.value=a),p.actions=[m],P(u),Ke({type:S.KEY,modifier:M,keycode:u})}function rd(e,t,a,n){const r=Qt[e],i=N(),o=U(),c=zt.has(e)?r:e,u=zt.has(e)?e:r,p=y=>{const h=Ee(i,o,y,d.layer);return(h.type===S.KEY&&h.modifier===M?Be(h.keycode):null)?.plugin===t?h.keycode:me()},m=p(c);oe(m).actions=[{type:"plugin",plugin:t,op:a,value:n.step}];const v=p(u);oe(v).actions=[{type:"plugin",plugin:t,op:a,value:-n.step}],P(m),P(v),ir(c,{type:S.KEY,modifier:M,keycode:m},u,{type:S.KEY,modifier:M,keycode:v})}function id(e){const t=Aa.find(o=>o.id===e),a=xe(),n=a===null?void 0:Qt[a];if(!t||a===null||n===void 0)return;const r=zt.has(a)?n:a,i=zt.has(a)?a:n;ir(r,{type:S.CONSUMER,modifier:0,keycode:t.up},i,{type:S.CONSUMER,modifier:0,keycode:t.down})}async function od(e){const t=Na();if(!t)return;let a;try{a=await window.orby.pickAppOrFile(e)}catch{g("El selector de archivos no está disponible","error");return}a?.ok&&(t.target=a.path,t.kind=e,P(A()),T())}async function sd(){const e=Na();if(!e)return;let t;try{t=await Ra()}catch{g("El detector de aplicaciones no está disponible","error");return}if(!t?.path){g("No se ha podido saber qué ejecutable es esa ventana","error");return}e.target=t.path,e.kind="app",P(A()),T(),g(`Añadida "${t.process||t.path}"`)}let At=[],ia=!1;function wo(){ia||At.length||(ia=!0,window.orby.listInstalledApps().then(e=>{At=e||[]}).catch(()=>{At=[]}).finally(()=>{ia=!1,Re()&&(d.tab==="app"||d.tab==="sequence")&&T()}))}function Mo(){return`<datalist id="installed-apps-list">
    ${At.map(e=>`<option value="${x(e.target)}" label="${x(e.name)}"></option>`).join("")}
  </datalist>`}function ld(e){const t=e===null?{target:"",kind:"app"}:L(e)?.actions?.[0]||{target:"",kind:"app"},a=t.kind==="file"?"file":"app";return a==="app"&&wo(),`
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
        ${Mo()}
        ${ia&&!At.length?'<span class="setting-desc">Buscando aplicaciones instaladas…</span>':""}
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

    <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar</button>`}function cd(e){const t=e===null?{text:""}:L(e)?.actions?.[0]||{text:""};return`
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

    <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar</button>`}async function dd(e=null,t="app"){let a;try{a=await window.orby.pickAppOrFile(t)}catch{g("El selector de archivos no está disponible","error");return}if(a?.ok)if(e!==null){const n=Q(e);n&&(n.target=a.path,n.kind=t,P(A()),T())}else Ne({type:"open_app",target:a.path,kind:t})}function ud(e,t){const a=Q(e);a&&(a.kind=t,P(A()),T())}async function pd(e){const t=Q(e);if(!t)return;let a;try{a=await Ra()}catch{g("El detector de aplicaciones no está disponible","error");return}if(!a?.path){g("No se ha podido saber qué ejecutable es esa ventana","error");return}t.target=a.path,t.kind="app",P(A()),T(),g(`Añadida "${a.process||a.path}"`)}function fd(e){const t=A();if(t===null)return;const a=L(t);a&&(a.actions.splice(e,1),hd(a),P(t),T())}function Dr(e,t){const a=A();if(a===null)return;const n=L(a);if(!n)return;const r=n.actions;if(!r[e]||r[e].type==="delay")return;let i=e+t;for(;r[i]&&r[i].type==="delay";)i+=t;r[i]&&([r[e],r[i]]=[r[i],r[e]],P(a),T())}function hd(e){const t=[];for(const a of e.actions)a.type==="delay"&&(!t.length||t[t.length-1].type==="delay")||t.push(a);t.length&&t[t.length-1].type==="delay"&&t.pop(),e.actions=t}function Q(e){const t=A();return(t===null?null:L(t))?.actions[e]||null}let be={modifier:0,keycode:0},Fe={x:0,y:0},Pe=null,ba=null;function md(){Pe&&(clearInterval(Pe),Pe=null)}function Hr(e=null){d.capturing="position",ba=e,T(),Pe&&clearInterval(Pe),Pe=setInterval(async()=>{try{const t=await window.orby.getMousePosition();Fe=t;const a=document.getElementById("seq-live-pos");a&&(a.textContent=`(${t.x}, ${t.y}) — mueve el ratón y pulsa Esc para fijarla`)}catch{}},80)}function mn(e){Pe&&(clearInterval(Pe),Pe=null),d.capturing=!1;const t=ba;if(ba=null,!e){T();return}if(t!==null){const a=Q(t);a&&(a.type="mouse_position",a.x=Fe.x,a.y=Fe.y,P()),T()}else Ne({type:"mouse_position",x:Fe.x,y:Fe.y})}function gd(){if(qe().modifier===M)return;const e=me();oe(e),Ke({type:S.KEY,modifier:M,keycode:e})}function ko(e){const t=e===null?null:L(e),a=t?.actions||[],n=d.capturing==="sequence",r=d.capturing==="position";a.some(y=>y.type==="open_app"&&y.kind!=="file")&&wo();const i={left:"Izquierdo",middle:"Central",right:"Derecho"},o=a.reduce((y,h,k)=>h.type==="delay"?y:k,-1),c=(y,h,k)=>`
    <button class="tool-btn small" data-act="seq-move-up" data-index="${y}" title="Subir paso" ${h?"disabled":""}>
      ${f("up",14)}
    </button>
    <button class="tool-btn small" data-act="seq-move-down" data-index="${y}" title="Bajar paso" ${k?"disabled":""}>
      ${f("down",14)}
    </button>`,u=(y,h)=>`
    <input type="number" class="text-input compact seq-count-input" min="1" max="99"
           data-act="seq-count" data-index="${h}" value="${y.count||1}" title="Veces">`,p=(y,h)=>y.count>1?`
    <input type="number" class="text-input compact seq-gap-input" min="0" max="32767" step="5"
           data-act="seq-gap-ms" data-index="${h}" value="${y.gap??to}" title="Espera entre repeticiones (ms)">`:"";let m=0;const v=a.map((y,h)=>{if(y.type==="delay")return`
        <li class="seq-item seq-gap">
          <span>${f("reset",13)} Espera</span>
          <span class="row-inline" style="gap:4px">
            <input type="number" class="text-input compact seq-gap-input" min="0" step="10"
                   value="${y.ms}" data-act="seq-delay-ms" data-index="${h}">
            <span class="setting-desc">ms</span>
          </span>
        </li>`;if(m++,y.type==="mouse_position")return`
        <li class="seq-item">
          <span>${m}. Posición de ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-x" data-index="${h}" value="${y.x}" title="x">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-y" data-index="${h}" value="${y.y}" title="y">
            <button class="tool-btn small" data-act="seq-recapture" data-index="${h}" title="Recapturar con el ratón">
              ${f("fit",14)}
            </button>
            ${c(h,m===1,h===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="center_mouse")return`
        <li class="seq-item">
          <span>${m}. Centrar ratón</span>
          <span class="row-inline" style="gap:4px">
            ${c(h,m===1,h===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="mouse_move")return`
        <li class="seq-item">
          <span>${m}. Mover ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dx" data-index="${h}" value="${y.dx}" title="dx">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dy" data-index="${h}" value="${y.dy}" title="dy">
            ${c(h,m===1,h===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`;if(y.type==="mouse_click"){const k=y.button||"left";return`
        <li class="seq-item">
          <span>${m}. Clic ${y.count>1?`×${y.count}`:""}</span>
          <span class="row-inline" style="gap:6px">
            <select class="select-input compact" data-act="seq-click-button" data-index="${h}">
              ${Object.entries(i).map(([B,F])=>`<option value="${B}" ${k===B?"selected":""}>${F}</option>`).join("")}
            </select>
            ${u(y,h)}
            ${p(y,h)}
            ${c(h,m===1,h===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`}if(y.type==="text")return`
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${m}. Escribir texto ${y.count>1?`×${y.count}`:""}</span>
            <span class="row-inline" style="gap:4px">
              ${u(y,h)}
              ${p(y,h)}
              ${c(h,m===1,h===o)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
                ${f("trash",14)}
              </button>
            </span>
          </div>
          <textarea class="text-input mt-4" rows="2" style="width:100%"
                    data-act="seq-text" data-index="${h}"
                    placeholder="Texto que se escribirá">${x(y.text||"")}</textarea>
        </li>`;if(y.type==="open_app"){const k=y.kind==="file"?"file":"app";return`
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${m}. Abrir</span>
            <span class="row-inline" style="gap:4px">
              ${c(h,m===1,h===o)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
                ${f("trash",14)}
              </button>
            </span>
          </div>
          <div class="row-inline mt-4" style="gap:6px">
            <button class="type-chip small ${k==="app"?"on":""}" data-act="seq-open-kind" data-index="${h}" data-kind="app">Aplicación</button>
            <button class="type-chip small ${k==="file"?"on":""}" data-act="seq-open-kind" data-index="${h}" data-kind="file">Archivo</button>
          </div>
          ${k==="app"?`
            <div class="row-inline mt-4" style="gap:6px">
              <button class="tool-btn small" data-act="seq-open-focus" data-index="${h}" title="Usar la app en primer plano ahora mismo">
                ${f("fit",14)} App en foco
              </button>
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${h}" data-kind="app" title="Examinar…">
                ${f("upload",14)}
              </button>
            </div>
            <input type="text" class="text-input compact mt-4" style="width:100%" list="installed-apps-list"
                   data-act="seq-open-target" data-index="${h}" value="${x(y.target||"")}"
                   placeholder="Escribe para buscar entre las instaladas" title="${x(y.target||"")}">
          `:`
            <div class="row-inline mt-4" style="gap:6px">
              <input type="text" class="text-input compact" style="flex:1;min-width:120px"
                     data-act="seq-open-target" data-index="${h}" value="${x(y.target||"")}"
                     placeholder="Ruta del archivo" title="${x(y.target||"")}">
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${h}" data-kind="file" title="Examinar…">
                ${f("upload",14)}
              </button>
            </div>
          `}
        </li>`}return y.type==="hotkey"?`
        <li class="seq-item">
          <span>${m}. ${x(Ot(y.modifier,y.keycode))} ${y.count>1?`×${y.count}`:""}</span>
          <span class="row-inline" style="gap:4px">
            ${u(y,h)}
            ${p(y,h)}
            ${c(h,m===1,h===o)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
              ${f("trash",14)}
            </button>
          </span>
        </li>`:`
      <li class="seq-item">
        <span>${m}. Tecla ${x(y.code)}</span>
        <span class="row-inline" style="gap:4px">
          ${c(h,m===1,h===o)}
          <button class="tool-btn danger small" data-act="seq-del" data-index="${h}" title="Quitar este paso">
            ${f("trash",14)}
          </button>
        </span>
      </li>`}).join("");return`
    <div class="field">
      <span class="field-label">Pasos de la secuencia</span>
      ${a.length?`<ul class="seq-list">${v}</ul>`:'<p class="setting-desc">Todavía no tiene ningún paso.</p>'}
      ${Mo()}

      <div class="row-inline mt-4">
        <button class="secondary-btn ${r?"is-capturing":""} ${H("pcSequences")?"":"unsupported"}"
                ${H("pcSequences")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-position">
          ${f("fit",16)} ${r?ba!==null?"Recapturando… (Esc fija)":"Capturando… (Esc fija)":"Posición de ratón"}
        </button>
        <button class="secondary-btn" data-act="seq-add-click">${f("bolt",16)} Clic</button>
        <button class="secondary-btn" data-act="seq-add-move">${f("reset",16)} Mover ratón</button>
        <button class="secondary-btn ${H("text")?"":"unsupported"}"
                ${H("text")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-text">${f("pencil",16)} Escribir texto</button>
        <button class="secondary-btn ${H("openApp")?"":"unsupported"}"
                ${H("openApp")?"":'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-open">${f("upload",16)} Abrir app/archivo</button>
      </div>
      ${r?`<p class="setting-desc" id="seq-live-pos">(${Fe.x}, ${Fe.y}) — mueve el ratón y pulsa Esc para fijarla</p>`:""}

      <span class="field-label mt-4">Tecla (con modificadores)</span>
      <div class="mod-grid">
        ${La.map(y=>`
          <button class="mod-chip ${be.modifier&y.bit?"on":""}"
                  data-act="seq-key-mod" data-bit="${y.bit}">${y.label}</button>`).join("")}
      </div>
      <div class="row-inline mt-4">
        <select class="select-input" data-act="seq-key-pick" style="flex:1">
          <option value="0" ${be.keycode?"":"selected"}>— ninguna —</option>
          ${lr(be.keycode)}
        </select>
        <button class="secondary-btn" data-act="seq-add-hotkey">${f("plus",16)} Añadir</button>
      </div>

      <button class="primary-btn full mt-4 ${n?"is-capturing":""}" data-act="seq-record">
        ${f("key",16)} ${n?"Grabando… pulsa teclas (Esc termina)":"Grabar secuencia de teclas"}
      </button>

      ${t?yd(t):""}

      <p class="setting-desc">
        "Grabar secuencia" solo reconoce letras, dígitos, Enter y Espacio; para el resto de
        teclas o combinaciones con modificadores usa "Tecla" arriba, y para meter un texto tal
        cual (correo, firma, un trozo de código) "Escribir texto". Entre cada dos pasos se
        espera automáticamente lo que pongas en "Espera" (${co} ms por
        defecto suele bastar).
      </p>

      <button class="secondary-btn full" data-act="seq-clear">${f("trash",16)} Quitar secuencia</button>
    </div>`}function yd(e){if(qt(e))return`<p class="setting-desc">${f("check",13)} Se ejecuta en el propio teclado: sigue
              funcionando aunque cierres esta app.</p>`;const t=(e.actions||[]).some(i=>i.type==="open_app"),a=(e.actions||[]).some(i=>i.type==="text"),n=(e.actions||[]).some(i=>i.type==="mouse_position"||i.type==="center_mouse");return`<p class="setting-desc">Se ejecuta en el PC, no en el teclado (${t?"abre una app o un archivo, algo que solo sabe hacer el PC":a?"escribe un texto, y eso el PC lo manda como unicode: el teclado solo sabe mandar códigos de tecla, que cambian con la distribución":n?"usa una posición de ratón absoluta, que de momento solo sabe reproducir el PC":`tiene más de ${Ji} pasos, o alguno de un formato antiguo`}): solo
            funciona con esta app abierta.</p>`}function Eo(){Tc({render:C,renderKeyGrid:Kr});const e=document.getElementById("profiles-body");e.addEventListener("click",bd),e.addEventListener("change",vd),e.addEventListener("input",$d),window.addEventListener("keydown",xd,!0),e.addEventListener("keydown",r=>{r.target.id==="variant-new-match"&&r.key==="Enter"&&(r.preventDefault(),Po())}),Vn(()=>{if(!Re())return;Kr();const r=document.getElementById("profile-bar");if(!r)return;const i=r.querySelectorAll(".tab-icon-canvas").length,o=s.profiles.filter((c,u)=>Hn(u)).length;i!==o?Io():Kt(r)}),gt().then(()=>{Re()&&d.selected&&C()}),Zn(()=>{Re()&&C()}),Ai(r=>{r==="applied"&&Re()&&C()}),window.orby.recorder.onState(r=>{Hc(r).catch(i=>console.error("Grabación:",i))});let t=-1,a=-1,n=-1;rt(()=>{s.profiles.length===t&&s.activeProfileIdx===a&&s.pageIdx===n||(t=s.profiles.length,a=s.activeProfileIdx,n=s.pageIdx,Re()&&C())})}function bd(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;if(a==="pick-profile")d.editingProfile=Number(t.dataset.idx),d.variantId=null,d.selected=null,C();else if(a==="pick-variant")d.variantId=t.dataset.id||null,d.selected=null,C();else if(a==="new-variant")wd();else if(a==="del-variant")kd();else if(a==="add-match")Po();else if(a==="add-match-current")Md();else if(a==="del-match")gl(d.variantId,t.dataset.match),C();else if(a==="clear-override")Ed();else if(a==="layer")d.layer=t.dataset.layer,d.selected=null,C();else if(a==="page")Ld(Number(t.dataset.page));else if(a==="page-add")Ad();else if(a==="page-del")Td(Number(t.dataset.page));else if(a==="pick-key")d.selected={kind:"key",index:Number(t.dataset.key)},d.capturing=!1,d.tab=Co(J()),C();else if(a==="pick-rotary")d.selected={kind:"rotary",slot:Number(t.dataset.slot)},d.capturing=!1,C();else if(a==="edit-icon")nn("view-oled",{profile:d.editingProfile,key:Number(t.dataset.key),layer:d.layer});else if(a==="copy-key")Nc();else if(a==="paste-key")Oc();else if(a==="rotary-type")Ke({type:Number(t.dataset.type),modifier:0,keycode:0});else if(a==="rotary-consumer")Ke({type:S.CONSUMER,modifier:0,keycode:Number(t.dataset.index)});else if(a==="rotary-consumer-pair")id(t.dataset.pair);else if(a==="rotary-mod"){const n=qe();Ke({type:S.KEY,modifier:n.modifier^Number(t.dataset.bit),keycode:n.keycode})}else if(a==="activate")si(d.editingProfile).then(()=>{s.activeProfileIdx=d.editingProfile,te(),C()}).catch(()=>g("El teclado no confirmó el cambio de perfil","error"));else if(a==="capture")d.capturing=!d.capturing,C();else if(a==="clear-action")Cd();else if(a==="set-goto-page")K(Je,Number(t.dataset.page));else if(a==="set-page-state")K(et,0);else if(a==="set-consumer")K(mt,Number(t.dataset.index));else if(a==="set-power")td(t.dataset.mode);else if(a==="toggle-mod"){const n=Number(t.dataset.bit),r=J();r.modifier>=mt||r.modifier===Je||r.modifier===et?K(n,0):K(r.modifier^n,r.keycode)}else if(a==="profile-new")Gr(null);else if(a==="profile-dup")Gr(d.editingProfile);else if(a==="profile-del")Pd();else if(a==="profile-icon")nn("view-oled",{profile:d.editingProfile,kind:"profile"});else if(a==="scroll-preset")yn({detentsPerRev:Number(t.dataset.value)});else if(a==="scroll-invert")yn({invert:!rr().invert});else if(a==="set-tab")Sd(t.dataset.tab);else if(a==="seq-add-position")d.capturing==="position"?mn(!1):Hr();else if(a==="seq-recapture")d.capturing==="position"?mn(!1):Hr(Number(t.dataset.index));else if(a==="seq-add-click")Ne({type:"mouse_click",button:"left"});else if(a==="seq-add-open")Ne({type:"open_app",target:"",kind:"app"});else if(a==="seq-add-text")Ne({type:"text",text:""});else if(a==="seq-open-browse")dd(Number(t.dataset.index),t.dataset.kind);else if(a==="seq-open-kind")ud(Number(t.dataset.index),t.dataset.kind);else if(a==="seq-open-focus")pd(Number(t.dataset.index));else if(a==="app-kind")ed(t.dataset.kind);else if(a==="app-browse")od(t.dataset.kind);else if(a==="app-focus")sd();else if(a==="seq-add-move")Ne({type:"mouse_move",dx:10,dy:0});else if(a==="seq-key-mod")be.modifier^=Number(t.dataset.bit),C();else if(a==="seq-add-hotkey"){if(!be.modifier&&!be.keycode)return;Ne({type:"hotkey",modifier:be.modifier,keycode:be.keycode}),be.keycode=0}else if(a==="seq-del")fd(Number(t.dataset.index));else if(a==="seq-move-up")Dr(Number(t.dataset.index),-1);else if(a==="seq-move-down")Dr(Number(t.dataset.index),1);else if(a==="seq-record")d.capturing=d.capturing==="sequence"?!1:"sequence",C();else if(a==="seq-clear")d.tab="shortcut",K(0,0);else if(a==="rotary-macro")gd();else if(a==="set-plugin")xo(t.dataset.plugin,t.dataset.op);else if(a==="rotary-plugin")hn(t.dataset.plugin,t.dataset.op);else if(a==="plugin-value-open"){const n=t.dataset.mode,r=qe(),i=n==="rotary"?r.type===S.KEY&&r.modifier===M?r.keycode:null:A(),o=i!=null?Be(i):null,c=o?.plugin===t.dataset.plugin&&o.op===t.dataset.op?o.value:void 0;Xc(t.dataset.plugin,t.dataset.op,n,c)}else if(a==="plugin-value-confirm")Zc();else if(a==="plugin-value-cancel")Qc();else if(a==="rotary-plugin-invert")nd();else if(a==="rotary-plugin-tab"){const n=t.dataset.plugin,r=qe();if((r.modifier===M?Be(r.keycode):null)?.plugin!==n){const o=!!uo(xe())?.part.discrete,c=Pa(Ze(n),o?"click":"turn")[0];c&&hn(n,c.op)}}else a==="rec-toggle"?Uc():a==="rec-mode"?Vc(t.dataset.mode):a==="rec-speed"?Fc(t.dataset.speed):a==="rec-clear"?Wc():a==="rec-stop"&&window.orby.recorder.stop()}function vd(e){const t=e.target.dataset.act;if(t==="pick-keycode"){const a=J(),r=a.modifier===mt||a.modifier===Je||a.modifier===et||a.modifier===M?0:a.modifier;K(r,Number(e.target.value))}else if(t==="rotary-keycode"){const a=qe();Ke({type:S.KEY,modifier:a.modifier,keycode:Number(e.target.value)})}else if(t==="scroll-slider")yn({detentsPerRev:Number(e.target.value)});else if(t==="variant-field")wr(d.variantId,{field:e.target.value});else if(t==="variant-name")wr(d.variantId,{name:e.target.value.trim()}),C();else if(t==="seq-click-button"){const a=Q(Number(e.target.dataset.index));a&&(a.button=e.target.value,P(A()))}else if(t==="seq-count"){const a=Q(Number(e.target.dataset.index));a&&(a.count=Math.max(1,Math.min(99,Math.round(Number(e.target.value))||1)),P(A()),C())}else if(t==="seq-delay-ms"){const a=Q(Number(e.target.dataset.index));a&&(a.ms=Math.max(0,Number(e.target.value)||0),P(A()))}else if(t==="seq-gap-ms"){const a=Q(Number(e.target.dataset.index));a&&(a.gap=Math.max(0,Math.min(32767,Math.round(Number(e.target.value))||0)),P(A()))}else if(t==="seq-pos-x"){const a=Q(Number(e.target.dataset.index));a&&(a.x=Math.round(Number(e.target.value)||0),P(A()))}else if(t==="seq-pos-y"){const a=Q(Number(e.target.dataset.index));a&&(a.y=Math.round(Number(e.target.value)||0),P(A()))}else if(t==="seq-open-target"){const a=Q(Number(e.target.dataset.index));a&&(a.target=e.target.value,P(A()))}else if(t==="seq-text"){const a=Q(Number(e.target.dataset.index));a&&(a.text=e.target.value,Wr(A()))}else if(t==="text-value"){const a=$o();a&&(a.text=e.target.value,Wr(A()))}else if(t==="app-target"){const a=Na();a&&(a.target=e.target.value,P(A()))}else if(t==="seq-move-dx"){const a=Q(Number(e.target.dataset.index));a&&(a.dx=Vr(e.target.value),P(A()))}else if(t==="seq-move-dy"){const a=Q(Number(e.target.dataset.index));a&&(a.dy=Vr(e.target.value),P(A()))}else t==="seq-key-pick"&&(be.keycode=Number(e.target.value))}function Vr(e){const t=Math.round(Number(e)||0);return Math.max(-127,Math.min(127,t))}let gn=null;function Fr(e){clearTimeout(gn),gn=setTimeout(()=>P(e),400)}function Wr(e){clearTimeout(gn),P(e),C()}let Ur=null;function $d(e){const t=e.target.dataset.act;if(t==="seq-text"){const a=Q(Number(e.target.dataset.index));a&&(a.text=e.target.value,Fr(A()))}else if(t==="text-value"){const a=$o();a&&(a.text=e.target.value,Fr(A()))}else if(t==="edit-name"){const a=e.target.value.slice(0,7);clearTimeout(Ur),Ur=setTimeout(async()=>{const n=N();if(!(!n||!Y())){n.name=a;try{await ci(d.editingProfile,a),z(),te(),Io()}catch{g("El teclado no confirmó el nombre","error")}}},250)}else if(t==="plugin-value-slider"){const a=Number(e.target.value);Yc(a);const n=document.getElementById("plugin-value-readout");n&&(n.textContent=a)}else if(t==="scroll-slider"){const a=Number(e.target.value),n=document.getElementById("scroll-value");n&&(n.textContent=a),To(a)}}function xd(e){if(!d.capturing||!d.selected)return;if(d.capturing==="position"){e.key==="Escape"&&(e.preventDefault(),e.stopPropagation(),mn(!0));return}if(e.preventDefault(),e.stopPropagation(),e.key==="Escape"){d.capturing=!1,C();return}if(d.capturing==="sequence"){const a=Ir(e);if(!a.keycode)return;Ne({type:"hotkey",modifier:a.modifier,keycode:a.keycode});return}const t=Ir(e);t.keycode&&(d.capturing=!1,d.selected.kind==="rotary"?Ke({type:S.KEY,modifier:t.modifier,keycode:t.keycode}):K(t.modifier,t.keycode))}async function wd(){let e="";try{e=((await Ra())?.process||"").toLowerCase()}catch{}const t=ml(d.editingProfile,{page:ya(),name:e?e.replace(/\.exe$/,""):"Variación",matches:e?[e]:[]});d.variantId=t.id,d.selected=null,C(),g(e?`Variación creada para "${e}"`:"Variación creada: indícale a qué app se aplica")}function Po(){const e=document.getElementById("variant-new-match");if(!e)return;const t=e.value.trim();if(t){if(!Ti(d.variantId,t)){g("Esa aplicación ya está en la lista","info");return}e.value="",C(),document.getElementById("variant-new-match")?.focus()}}async function Md(){try{const t=((await Ra())?.process||"").toLowerCase();if(!t){g("Aún no se ha detectado ninguna ventana","error");return}if(!Ti(d.variantId,t)){g("Esa aplicación ya está en la lista","info");return}C()}catch{g("El detector de aplicaciones no está disponible","error")}}async function kd(){const e=U();e&&confirm(`Se borrará la variación "${e.name}" y sus ${Rt(e)} cambios.

El perfil base no se toca.

¿Continuar?`)&&(await yl(e.id),d.variantId=null,d.selected=null,C(),g("Variación eliminada"))}async function Ed(){const e=U();if(!e||!d.selected)return;const t=d.selected.kind==="rotary"?"rotary":"keys",a=d.selected.kind==="rotary"?pe(d.selected.slot,d.layer):le(d.selected.index,d.layer);if(Mr(e.id,t,a),d.selected.kind==="key"&&Mr(e.id,"labels",V(d.selected.index,d.layer)),Ae(e.id))try{if(t==="keys"){await _a(e.id,"keys",a);const n=V(d.selected.index,d.layer);n>=0&&await _a(e.id,"labels",n)}else await _a(e.id,"rotary",a)}catch{g("El teclado no confirmó la vuelta al valor base","error")}d.selected.kind==="key"&&(d.tab=Co(J())),C()}async function Gr(e){if(!d.busy){if(!s.connected){g("Teclado no conectado","error");return}if(s.profiles.length>=s.maxProfiles){g(`El teclado admite como máximo ${s.maxProfiles} perfiles`,"error");return}d.busy=!0,C();try{const t=e===null?await di():await gs(e);Ma(),await fe(),d.editingProfile=Number.isInteger(t)?t:s.profiles.length-1,d.selected=null,z(),g(e===null?"Perfil creado":"Perfil duplicado")}catch(t){g(`No se pudo crear el perfil: ${t.message}`,"error")}finally{d.busy=!1,C()}}}async function Pd(){if(d.busy)return;if(!s.connected){g("Teclado no conectado","error");return}if(s.profiles.length<=1){g("Tiene que quedar al menos un perfil","error");return}const e=N();if(confirm(`Se borrará el perfil "${e?.name||""}" con sus atajos, mandos e iconos.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`)){d.busy=!0,C();try{await ys(d.editingProfile),vl(d.editingProfile),d.variantId=null,Ma(),await fe(),d.editingProfile=Math.min(d.editingProfile,s.profiles.length-1),d.selected=null,z(),g("Perfil eliminado")}catch(t){g(`No se pudo eliminar: ${t.message}`,"error")}finally{d.busy=!1,C()}}}async function Cd(){const e=J();if(e.modifier===M&&It(e.keycode)){const t=lt(e.keycode);t&&await bo(t.id)}await K(0,0),await Rc()}function Co(e){return e.modifier===mt?"media":e.modifier===Je||e.modifier===et?"pages":e.modifier===M?It(e.keycode)?"record":Yt(e.keycode)||$t(e.keycode)?"media":ha(e.keycode)?"text":fa(e.keycode)?"app":"sequence":"shortcut"}function Sd(e){const t=d.tab;if(d.tab=e,t==="record"&&e!=="record"){const a=J();if(a.modifier===M&&It(a.keycode)){const n=lt(a.keycode);n&&bo(n.id)}}if(e==="sequence"&&t!=="sequence"){const a=J();if(a.modifier!==M||fa(a.keycode)||Yt(a.keycode)||$t(a.keycode)||ha(a.keycode)||It(a.keycode)){const n=me();oe(n),K(M,n);return}}if(e==="text"&&t!=="text"){const a=J();if(a.modifier!==M||!ha(a.keycode)){const n=me(),r=oe(n);r.actions=[{type:"text",text:""}],P(n),K(M,n);return}}if(e==="app"&&t!=="app"){const a=J();if(a.modifier!==M||!fa(a.keycode)){const n=me(),r=oe(n);r.actions=[{type:"open_app",target:""}],P(n),K(M,n);return}}if(e==="record"&&t!=="record"){const a=J();if(a.modifier!==M||!It(a.keycode)){Kc();return}}C()}async function yn(e){const t=N();if(!t||!Y())return;const a=bi(d.layer),n={...Rn(t,d.layer),...e};t.scroll[a]=n,d.editingProfile===s.activeProfileIdx&&a===(s.superActive?1:0)&&(s.scroll={...s.scroll,...n}),C();try{await li(d.editingProfile,a,n.detentsPerRev,n.invert),z()}catch{g("El teclado no confirmó la calibración de la rueda","error")}}function C(){const e=document.getElementById("profiles-body");if(!e)return;const t=e.querySelector(".editor-main")?.scrollTop,a=e.querySelector(".editor-inspector")?.scrollTop;if(d.capturing!=="position"&&md(),!s.profiles.length){e.innerHTML=`<div class="empty-panel glass-panel">
      ${f("plug",40)}
      <h3>Sin perfiles cargados</h3>
      <p>Conecta el Orby una vez: a partir de ahí queda una copia en el PC y podrás
         editarlos aunque no lo tengas enchufado.</p>
    </div>`;return}d.editingProfile>=s.profiles.length&&(d.editingProfile=0);const n=N();if(e.innerHTML=`
    <div class="profile-bar" id="profile-bar">${So()}</div>
    ${Id()}

    <div class="editor-layout">
      <div class="editor-main">
        ${Od()}
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
              ${Rd()}
            </div>
          </div>
          ${Nd()}

          <div class="okey-grid" id="profile-key-grid">${Lo()}</div>
          <p class="grid-status" id="profile-grid-status"></p>
        </div>

        <div class="glass-panel oled-card">
          <div class="card-header">${f("reset",20)}<h2>Encoders</h2></div>
          <div class="rotary-groups">${_d()}</div>
          <p class="setting-desc mt-4">
            Cada capa guarda sus propias acciones: con <strong>SUPER</strong> mantenida los encoders
            hacen lo que configures aquí en la capa SUPER. Dentro del menú del teclado siguen
            sirviendo para navegar.
          </p>
        </div>

        ${zd()}
      </div>

      ${Bd()}
    </div>`,Kt(e),Ao(),To(rr().detentsPerRev),je(d.editingProfile),t!==void 0){const r=e.querySelector(".editor-main");r&&(r.scrollTop=t)}if(a!==void 0){const r=e.querySelector(".editor-inspector");r&&(r.scrollTop=a)}}function So(){const e=s.profiles.length>=s.maxProfiles,t=st(s.deviceInfo,"profileIcon");return`
    <div class="profile-tabs">
      ${s.profiles.map((a,n)=>`
        <button class="profile-tab ${n===d.editingProfile?"active":""} ${n===s.activeProfileIdx?"is-live":""}"
                data-act="pick-profile" data-idx="${n}">
          <span class="tab-icon" style="--accent:${sa(n).accent}">${Hn(n)?`<canvas class="tab-icon-canvas" data-bmp="${n}:0:20"></canvas>`:f(sa(n).icon,18)}</span>
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
              title="${t?"Icono que enseña este perfil en el menú del teclado":`Necesita firmware ${er.profileIcon.since} o posterior`}">
        ${f("oled",16)} Icono del perfil
      </button>
      <span class="profile-count">${s.profiles.length} / ${s.maxProfiles}</span>
    </div>`}function Io(){const e=document.getElementById("profile-bar");e&&(e.innerHTML=So(),Kt(e))}function Id(){const e=fl(d.editingProfile,ya()),t=s.profiles[d.editingProfile],a=nt()&&t&&D(t)>1?` · página ${ya()+1}`:"";return`
    <div class="variant-bar">
      <span class="variant-bar-label">${f("bolt",14)} Según la app${a}</span>
      <div class="chip-row">
        <button class="chip ${d.variantId?"":"on"}" data-act="pick-variant" data-id="">
          Perfil base
        </button>
        ${e.map(n=>`
          <button class="chip ${d.variantId===n.id?"on":""} ${Ae(n.id)?"is-live":""}"
                  data-act="pick-variant" data-id="${n.id}"
                  title="${x(n.matches.length?`Se aplica con: ${n.matches.join(", ")}`:"Sin aplicaciones asignadas")}">
            ${x(n.name)}
            <em class="chip-count">${Rt(n)}</em>
          </button>`).join("")}
        <button class="chip ghost" data-act="new-variant">${f("plus",14)} Nueva variación</button>
      </div>
    </div>`}async function Ld(e){const t=s.profiles[d.editingProfile];if(!(!t||e===(t.pageIdx||0))){if(s.connected&&d.editingProfile!==s.activeProfileIdx){g("Para editar otra página, activa antes este perfil en el teclado");return}if(d.selected=null,d.variantId=null,!s.connected){t.pageIdx=e,C();return}try{await vi(e),je(d.editingProfile),C()}catch(a){g(`No he podido cambiar de página: ${a.message}`),C()}}}async function Ad(){if(!(!s.profiles[d.editingProfile]||!Y())){if(d.editingProfile!==s.activeProfileIdx){g("Activa este perfil en el teclado para añadirle páginas");return}try{if(await Rs()){const t=s.profiles[d.editingProfile];await vi(D(t)-1),je(d.editingProfile),g(`Página ${D(t)} añadida, vacía`)}else g(`El tope es de ${Wt()} páginas por perfil`);C()}catch(t){g(`No he podido añadir la página: ${t.message}`)}}}async function Td(e){const t=s.profiles[d.editingProfile];if(!(!t||D(t)<=1||!Y())){if(d.editingProfile!==s.activeProfileIdx){g("Activa este perfil en el teclado para borrarle páginas");return}if(confirm(`¿Eliminar la página ${e+1} de ${x(t.name)}?

Se van sus teclas, etiquetas, mandos e iconos. Las páginas siguientes se recolocan.`))try{await Ns(e),bl(d.editingProfile,e),je(d.editingProfile),d.selected=null,d.variantId=null,g(`Página ${e+1} eliminada`),C()}catch(a){g(`No he podido eliminar la página: ${a.message}`)}}}function Rd(){if(!nt())return"";const e=s.profiles[d.editingProfile];if(!e)return"";const t=D(e),a=e.pageIdx||0;let n="";for(let i=0;i<t;i++)n+=`<button class="toggle-btn ${i===a?"active":""}"
                     data-act="page" data-page="${i}" title="Página ${i+1}">${i+1}</button>`;const r=t<Wt();return`
    <div class="layer-toggle page-toggle">
      ${n}
      ${r?`<button class="toggle-btn page-add" data-act="page-add"
                          title="Añadir una página copiando la actual">+</button>`:""}
      ${t>1?`<button class="toggle-btn page-del" data-act="page-del" data-page="${a}"
                             title="Eliminar la página ${a+1}">−</button>`:""}
    </div>`}function Nd(){if(!nt())return"";const e=s.profiles[d.editingProfile];if(!e)return"";const t=D(e);if(t<=1)return`<p class="grid-status">Este perfil tiene una sola página. Con
      <strong>+</strong> añades otra en blanco, y el teclado alterna entre ellas
      con una pulsación corta del botón de menú.</p>`;const a=d.editingProfile===s.activeProfileIdx;return`<p class="grid-status">Editando la <strong>página ${(e.pageIdx||0)+1}</strong>
    de ${t}. Cada página tiene sus teclas, etiquetas, mandos, rueda e iconos.
    ${a?"La página que elijas aquí es la que se pone en el teclado, así que las pantallas enseñan lo que estás tocando.":""}</p>`}function Od(){const e=U();if(!e)return"";const t=Ae(e.id),a=Rt(e);return`
    <div class="glass-panel oled-card variant-card">
      <div class="card-header">
        ${f("bolt",20)}<h2>Variación de ${x(N().name)}</h2>
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
    </div>`}function Lo(){const e=N(),t=U(),a=Te();let n="";for(let r=0;r<12;r++){const i=_e[r],o=V(r,d.layer),c=qi(e,t,r,d.layer),u=o>=0?_i(e,t,r,d.layer):"",p=o>=0?ye(d.editingProfile,o):null,m=c.modifier||c.keycode,v=io(c),y=t&&(he(t,"keys",le(r,d.layer))||he(t,"labels",o));n+=`
      <button class="okey ${a===r?"selected":""} ${p?"has-icon":""} ${i?"":"roleless"} ${y?"is-override":""}"
              data-act="pick-key" data-key="${r}">
        <span class="okey-num">T${r+1}${i?`<em>P${i}</em>`:""}${y?'<i class="okey-dot" title="Cambiado en esta variación"></i>':""}</span>
        <span class="okey-screen">
          ${i?p?`<canvas class="okey-canvas" data-bmp="${ue(d.editingProfile,o)}"></canvas>`:`<span class="okey-text">${x(u||"—")}</span>`:`<span class="okey-role">${r+1===ft?"SUPER":"MENÚ"}</span>`}
        </span>
        <span class="okey-action ${m?"assigned":""}">${x(hc(c))}${v?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esta tecla no hace nada">${f("plug",11)}</i>`:""}</span>
      </button>`}return n}function Ao(){const e=document.getElementById("profile-key-grid");e&&Kt(e);const t=document.getElementById("profile-grid-status");t&&(t.textContent=el()?"Leyendo iconos del teclado…":`Capa ${d.layer==="super"?"SUPER":"NORMAL"} · las teclas ${en} y ${ft} no tienen pantalla: la ${ft} es el modificador SUPER y la ${en} abre el menú al mantenerla.`)}function Kr(){const e=document.getElementById("profile-key-grid");e&&(e.innerHTML=Lo(),Ao())}function qd(e){return e?.type===S.KEY&&e.modifier===M&&$t(e.keycode)}function bn(e){return!e?.type||tr(e.type)||qd(e)?!0:e.type===S.CONSUMER&&!!rc(e.keycode)}function _d(){const e=N(),t=U(),a=xe(),n=(r,i,o)=>`
    <button class="rotary-part ${a===r.slot?"selected":""} ${i.type?"assigned":""} ${o?"is-override":""}"
            data-act="pick-rotary" data-slot="${r.slot}">
      <span class="rp-dir">${r.short}</span>
      <span class="rp-body">
        <em>${r.label}</em>
        <strong>${x(_t(i))}${pn(i)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong>
      </span>
    </button>`;return lo.map(r=>{const[i,o,c]=r.parts,u=Ee(e,t,i.slot,d.layer),p=Ee(e,t,o.slot,d.layer),m=bn(u)&&bn(p),v=t&&he(t,"rotary",pe(i.slot,d.layer)),y=t&&he(t,"rotary",pe(o.slot,d.layer)),h=Ee(e,t,c.slot,d.layer),k=t&&he(t,"rotary",pe(c.slot,d.layer)),B=m?n({slot:i.slot,label:"Giro",short:"⟳"},u,v):n(i,u,v)+n(o,p,y);return`
    <div class="rotary-group">
      <span class="rotary-group-name">${f(r.icon,14)} ${r.name}</span>
      ${B}
      ${n(c,h,k)}
    </div>`}).join("")}function zd(){const e=rr(),t=d.layer==="super"?"SUPER":"normal",a=N(),n=U(),r=Ee(a,n,R.WHEEL_CW,d.layer),i=Ee(a,n,R.WHEEL_CCW,d.layer);return`
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
            ${Lc.map(o=>`
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
              <span class="rp-body"><em>Hacia abajo</em><strong>${x(_t(r))}${pn(r)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong></span>
            </button>
            <button class="rotary-part ${xe()===R.WHEEL_CCW?"selected":""}"
                    data-act="pick-rotary" data-slot="${R.WHEEL_CCW}">
              <span class="rp-dir">↑</span>
              <span class="rp-body"><em>Hacia arriba</em><strong>${x(_t(i))}${pn(i)?`<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${f("plug",11)}</i>`:""}</strong></span>
            </button>
          </div>

          <p class="setting-desc">
            La sensibilidad pertenece a <strong>${x(N().name)}</strong> en la capa
            ${t}: cada perfil —y cada capa— puede tener la suya. El dibujo de la rueda en
            pantalla (forma del marcador, sentido, desfase) se calibra en <strong>Ajustes</strong>.
            ${s.scroll.hires?"Con el multiplicador negociado el desplazamiento viaja en unidades de 1/120 de clic.":"Hasta que Windows pida el multiplicador, las aplicaciones antiguas saltarán de tres en tres líneas."}
          </p>
        </div>
      </div>
    </div>`}function To(e){const t=document.getElementById("scroll-derived");if(!t)return;const a=360/e,n=e*120/4096;t.innerHTML=`
    <li><span class="lbl">Giro por clic</span><span class="val">${a.toFixed(1)}°</span></li>
    <li><span class="lbl">Unidades HID por cuenta</span><span class="val">${n.toFixed(3)}</span></li>
    <li><span class="lbl">Líneas por vuelta (Windows)</span><span class="val">${(e*3).toFixed(0)}</span></li>`}function Bd(){return d.selected?d.selected.kind==="rotary"?jd():Dd():`<div class="editor-inspector glass-panel empty-panel">
      ${f("key",36)}
      <h3>Elige una tecla o un mando</h3>
      <p>Selecciona cualquier tecla para cambiar su icono, su etiqueta y su atajo, o un giro
         de los encoders y la rueda para reasignarlo.</p>
    </div>`}function jd(){const e=xe(),t=uo(e),a=qe(),n=!!t?.part.discrete,r=a.type===S.KEY&&a.modifier===M,i=n?"click":"turn",o=r?Be(a.keycode):null,c=!!o,u=c?Ze(o.plugin):null,p=Pa(u,i),m=c&&!n&&!!Number(o.value),v=m&&ad(e,o),y=Ic.filter(E=>!(n&&E.turnOnly)),h=U(),k=!!(h&&he(h,"rotary",pe(e,d.layer))),B=N().rotary?.[pe(e,d.layer)]||{type:0,modifier:0,keycode:0},F=!n&&bn(a)?"Giro":t?.part.label;return`
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${x(t?.group.name||"Mando")}</h3>
        <span class="pill">${d.layer==="super"?"Capa SUPER":"Capa normal"}</span>
      </div>
      <p class="setting-desc inspector-sub">${x(F||"")}</p>

      ${h?`
        <div class="override-note ${k?"is-override":""}">
          <span>${k?`Cambiado en <strong>${x(h.name)}</strong> · el perfil base hace
               <code>${x(_t(B))}</code>`:`Editando <strong>${x(h.name)}</strong>: solo valdrá con esa app`}</span>
          ${k?`<button class="secondary-btn" data-act="clear-override">
                         ${f("reset",14)} Volver al valor base
                       </button>`:""}
        </div>`:""}

      <div class="field">
        <span class="field-label">Tipo de acción</span>
        <div class="type-grid">
          ${y.map(E=>`
            <button class="type-chip ${a.type===E.type&&!(E.type===S.KEY&&r)?"on":""}"
                    data-act="rotary-type" data-type="${E.type}">${E.label}</button>`).join("")}
          <button class="type-chip ${r&&!c?"on":""}" data-act="rotary-macro">Secuencia</button>
          ${Wi(i).map(E=>`
            <button class="type-chip ${u?.id===E.id?"on":""}"
                    data-act="rotary-plugin-tab" data-plugin="${E.id}">${x(E.name)}</button>`).join("")}
        </div>
      </div>

      ${c?`
        <div class="field">
          <span class="field-label">Qué controla</span>
          ${p.length?`
            <div class="consumer-grid">
              ${p.map(E=>`
                <button class="consumer-chip ${o.op===E.op?"on":""}"
                        data-act="${E.value?"plugin-value-open":"rotary-plugin"}" data-mode="rotary"
                        data-plugin="${o.plugin}"
                        data-op="${x(E.op)}">${x(E.label)}</button>`).join("")}
            </div>
            ${ee.open&&ee.mode==="rotary"&&ee.plugin===o.plugin?Ro(u,ee.op):""}`:`
            <p class="setting-desc">
              ${u?"Este complemento no ofrece nada para este mando.":`El complemento «${x(o.plugin)}» no está instalado o está
                   desactivado: el mando no hará nada hasta que vuelva.`}
            </p>`}
          ${m?`
            <button class="consumer-chip ${v?"on":""}" data-act="rotary-plugin-invert"
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

      ${a.type===S.CONSUMER?`
        <div class="field">
          <span class="field-label">Acción</span>
          <div class="consumer-grid">
            ${n?Xt.map(E=>`
                  <button class="consumer-chip ${a.keycode===E.index?"on":""}"
                          data-act="rotary-consumer" data-index="${E.index}">${E.label}</button>`).join(""):nc.map(E=>E.pairId?`<button class="consumer-chip ${a.keycode===E.up||a.keycode===E.down?"on":""}"
                             data-act="rotary-consumer-pair" data-pair="${E.pairId}">${E.label}</button>`:`<button class="consumer-chip ${a.keycode===E.index?"on":""}"
                             data-act="rotary-consumer" data-index="${E.index}">${E.label}</button>`).join("")}
          </div>
          ${n?"":`
            <p class="setting-desc">
              Cada muesca sube o baja un paso, y el sentido lo pone el propio giro: no hace
              falta elegir qué va en cada lado.
            </p>`}
        </div>`:""}

      ${r&&!c?ko(a.keycode):""}

      ${a.type===S.KEY&&!r?`
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${La.map(E=>`
              <button class="mod-chip ${a.modifier&E.bit?"on":""}"
                      data-act="rotary-mod" data-bit="${E.bit}">${E.label}</button>`).join("")}
          </div>
        </div>
        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="rotary-keycode">
            <option value="0" ${a.keycode?"":"selected"}>— ninguna —</option>
            ${lr(a.keycode)}
          </select>
        </label>
        <button class="primary-btn full ${d.capturing?"is-capturing":""}" data-act="capture">
          ${f("key",16)} ${d.capturing?"Pulsa el atajo… (Esc cancela)":"Capturar atajo del teclado"}
        </button>`:""}

      ${tr(a.type)?`
        <p class="setting-desc">
          El sentido va implícito en el giro, así que esta acción cubre las dos direcciones.
          ${a.type===S.SCROLL_V?"En la rueda magnética es la única opción que aprovecha la alta resolución; el resto trabajan por clics completos.":""}
        </p>`:""}

      <div class="inspector-summary">
        <span class="field-label">Resultado</span>
        <code>${x(_t(a))}</code>
      </div>
    </div>`}function Dd(){const e=Te(),t=N(),a=U(),n=V(e,d.layer),r=J(),i=r.modifier===mt,o=r.modifier===M,c=i||o||r.modifier===Je||r.modifier===et,u=lr(c?0:r.keycode),p=n>=0&&!!ye(d.editingProfile,n),m=t.keys[le(e,d.layer)]||{modifier:0,keycode:0},v=!!(a&&(he(a,"keys",le(e,d.layer))||he(a,"labels",n))),y=d.tab;return`
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
               <code>${x(Ot(m.modifier,m.keycode))}</code>`:`Editando <strong>${x(a.name)}</strong>: lo que cambies aquí solo
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
          Esta tecla no tiene pantalla: la ${ft} es el modificador SUPER y la ${en}
          abre el menú del teclado al mantenerla, así que el firmware no ejecuta su atajo.
        </p>`}

      <div class="inspector-tabs">
        ${[{id:"shortcut",label:"Atajo",cap:null},{id:"sequence",label:"Secuencia",cap:null},{id:"text",label:"Texto",cap:"text"},{id:"record",label:"Grabar",cap:"recorder"},{id:"app",label:"App",cap:"openApp"},{id:"media",label:"Multimedia",cap:null}].map(({id:h,label:k,cap:B})=>{const F=B&&!H(B);return`<button class="inspector-tab ${y===h?"active":""} ${F?"unsupported":""}"
                          ${F?'disabled title="Necesita OrbyGUI de escritorio"':""}
                          data-act="set-tab" data-tab="${h}">${k}</button>`}).join("")}
        ${nt()?`<button class="inspector-tab ${y==="pages"?"active":""}" data-act="set-tab" data-tab="pages">Páginas</button>`:""}
      </div>

      ${y==="shortcut"?`
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${La.map(h=>`
              <button class="mod-chip ${!c&&r.modifier&h.bit?"on":""}"
                      data-act="toggle-mod" data-bit="${h.bit}">${h.label}</button>`).join("")}
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

      ${y==="sequence"?ko(o?r.keycode:null):""}

      ${y==="text"?cd(o?r.keycode:null):""}

      ${y==="record"?Gc(o?r.keycode:null):""}

      ${y==="app"?ld(o?r.keycode:null):""}

      ${y==="media"?`
        <div class="field">
          <span class="field-label">Acción multimedia</span>
          <div class="consumer-grid consumer-grid-3col">
            ${Xt.map(h=>`
              <button class="consumer-chip ${i&&r.keycode===h.index?"on":""}"
                      data-act="set-consumer" data-index="${h.index}">${h.label}</button>`).join("")}
          </div>
        </div>
        <div class="field">
          <span class="field-label">Energía del PC</span>
          <div class="consumer-grid">
            ${Object.entries(eo).map(([h,k])=>`
              <button class="consumer-chip ${o&&Yt(r.keycode)&&L(r.keycode).actions[0].mode===h?"on":""}"
                      data-act="set-power" data-mode="${h}">${k}</button>`).join("")}
          </div>
          <p class="setting-desc">
            Esta acción la ejecuta el PC, no el teclado, así que OrbyGUI tiene que estar abierto
            —vale con el icono de la bandeja— para que funcione.
          </p>
        </div>
        ${Hd(o?r.keycode:null)}`:""}

      ${y==="pages"?Fd(r):""}
    </div>`}function Hd(e){const t=e===null?null:Be(e),a=ee;return Wi("key").map(n=>`
    <div class="field">
      <span class="field-label">${x(n.name)}</span>
      <div class="consumer-grid">
        ${Pa(n,"key").map(r=>`
          <button class="consumer-chip ${t?.plugin===n.id&&t.op===r.op?"on":""}"
                  data-act="${r.value?"plugin-value-open":"set-plugin"}" data-mode="key"
                  data-plugin="${n.id}" data-op="${x(r.op)}">
            ${x(r.label)}
          </button>`).join("")}
      </div>
      ${a.open&&a.mode==="key"&&a.plugin===n.id?Ro(n,a.op):""}
      <p class="setting-desc">
        ${x(n.description||"Lo ejecuta el PC, así que OrbyGUI tiene que estar abierto.")}
        Lo que necesite sentido de giro (subir, bajar) se asigna a un mando, no a una tecla.
      </p>
    </div>`).join("")+Vd(e)}function Ro(e,t){const{action:a}=Ca(e.id,t);if(!a?.value)return"";const{min:n,max:r,step:i}=a.value,o=ee.value;return`
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
    </div>`}function Vd(e){const t=e===null?null:Be(e);if(!(_e[Te()]>0))return"";const n=Gl().map(r=>`
    <div class="field">
      <span class="field-label">${x(r.name)} · pantalla</span>
      <div class="consumer-grid">
        ${Sa(r).map(i=>`
          <button class="consumer-chip ${t?.plugin===r.id&&t.op===i.op?"on":""}"
                  data-act="set-plugin" data-plugin="${r.id}" data-op="${x(i.op)}">
            ${x(i.label)}
          </button>`).join("")}
      </div>
    </div>`).join("");return n?`${n}
    <p class="setting-desc">
      La tecla deja de hacer nada al pulsarla: solo enseña el valor en su pantalla, y se
      actualiza solo cada par de segundos mientras esta página esté puesta en el teclado.
    </p>`:""}function Fd(e){if(!nt())return"";const t=s.profiles[d.editingProfile],a=t?D(t):1,n=e.modifier===Je,r=e.modifier===et;let i="";for(let o=1;o<=Wt();o++){const c=o>a;i+=`<button class="consumer-chip ${n&&e.keycode===o?"on":""}"
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
    </div>`}const Wd=Object.freeze(Object.defineProperty({__proto__:null,init:Eo,render:C,syncAllMacrosToDevice:no},Symbol.toStringTag,{value:"Module"})),b='fill="currentColor" stroke="none"',Ud=[{id:"basic",label:"Formas y flechas"},{id:"media",label:"Multimedia"},{id:"edit",label:"Edición"},{id:"system",label:"Sistema"},{id:"dev",label:"Programación"},{id:"comm",label:"Comunicación"},{id:"misc",label:"Varios"}],Gd={basic:[["arrow-up","Flecha arriba","subir norte",'<path d="M12 20V4M5 11l7-7 7 7"/>'],["arrow-down","Flecha abajo","bajar sur",'<path d="M12 4v16M5 13l7 7 7-7"/>'],["arrow-left","Flecha izquierda","atras oeste",'<path d="M20 12H4M11 5l-7 7 7 7"/>'],["arrow-right","Flecha derecha","siguiente este",'<path d="M4 12h16M13 5l7 7-7 7"/>'],["arrow-ul","Flecha arriba izquierda","diagonal",'<path d="M18 18L6 6M6 14V6h8"/>'],["arrow-ur","Flecha arriba derecha","diagonal",'<path d="M6 18L18 6M18 14V6h-8"/>'],["arrow-dl","Flecha abajo izquierda","diagonal",'<path d="M18 6L6 18M6 10v8h8"/>'],["arrow-dr","Flecha abajo derecha","diagonal",'<path d="M6 6l12 12M18 10v8h-8"/>'],["chevron-up","Punta arriba","desplegar",'<path d="M5 15l7-7 7 7"/>'],["chevron-down","Punta abajo","desplegar menu",'<path d="M5 9l7 7 7-7"/>'],["chevron-left","Punta izquierda","anterior",'<path d="M15 5l-7 7 7 7"/>'],["chevron-right","Punta derecha","siguiente",'<path d="M9 5l7 7-7 7"/>'],["chevrons-left","Doble punta izquierda","inicio retroceder",'<path d="M11 18l-6-6 6-6M19 18l-6-6 6-6"/>'],["chevrons-right","Doble punta derecha","fin avanzar",'<path d="M13 6l6 6-6 6M5 6l6 6-6 6"/>'],["arrows-h","Flechas horizontales","ancho ajustar",'<path d="M3 12h18M6.5 8.5L3 12l3.5 3.5M17.5 8.5L21 12l-3.5 3.5"/>'],["arrows-v","Flechas verticales","alto ajustar",'<path d="M12 3v18M8.5 6.5L12 3l3.5 3.5M8.5 17.5L12 21l3.5-3.5"/>'],["rotate-cw","Girar a la derecha","rehacer recargar",'<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>'],["rotate-ccw","Girar a la izquierda","deshacer recargar",'<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>'],["shuffle","Aleatorio","mezclar random",'<path d="M3 6h4l10 12h4M3 18h4l2.5-3M14.5 8.5L17 6h4"/><path d="M18 3l3 3-3 3M18 15l3 3-3 3"/>'],["repeat","Repetir","bucle loop",'<path d="M4 10V8a3 3 0 0 1 3-3h13"/><path d="M17 2l3 3-3 3"/><path d="M20 14v2a3 3 0 0 1-3 3H4"/><path d="M7 22l-3-3 3-3"/>'],["plus","Más","añadir sumar nuevo",'<path d="M12 4v16M4 12h16"/>'],["minus","Menos","quitar restar",'<path d="M4 12h16"/>'],["close","Cerrar","equis cancelar salir",'<path d="M5 5l14 14M19 5L5 19"/>'],["check","Confirmar","ok aceptar tick visto",'<path d="M4 12.5l5.5 5.5L20 6"/>'],["check-circle","Confirmado","ok aceptar",'<circle cx="12" cy="12" r="9"/><path d="M7.5 12.5l3 3 6-6.5"/>'],["x-circle","Cancelar","error borrar",'<circle cx="12" cy="12" r="9"/><path d="M8.5 8.5l7 7M15.5 8.5l-7 7"/>'],["circle","Círculo","redondo",'<circle cx="12" cy="12" r="8"/>'],["circle-fill","Círculo macizo","punto bola",`<circle cx="12" cy="12" r="8" ${b}/>`],["square","Cuadrado","caja",'<rect x="4" y="4" width="16" height="16" rx="2"/>'],["square-fill","Cuadrado macizo","caja bloque",`<rect x="4" y="4" width="16" height="16" rx="2" ${b}/>`],["triangle","Triángulo","aviso",'<path d="M12 4l9 16H3z"/>'],["triangle-fill","Triángulo macizo","aviso",`<path d="M12 4l9 16H3z" ${b}/>`],["star","Estrella","favorito destacar",'<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z"/>'],["star-fill","Estrella maciza","favorito destacar",`<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z" ${b}/>`],["heart","Corazón","favorito megusta",'<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z"/>'],["heart-fill","Corazón macizo","favorito megusta",`<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z" ${b}/>`],["diamond","Rombo","diamante",'<path d="M12 3l9 9-9 9-9-9z"/>'],["hexagon","Hexágono","panal",'<path d="M12 2.5l8 4.6v9.8l-8 4.6-8-4.6V7.1z"/>']],media:[["play","Reproducir","play iniciar",`<path d="M7 4.5l12 7.5-12 7.5z" ${b}/>`],["play-circle","Reproducir en círculo","play",`<circle cx="12" cy="12" r="9"/><path d="M10 8.5l6 3.5-6 3.5z" ${b}/>`],["pause","Pausa","pausar",`<rect x="6" y="4" width="4" height="16" rx="1" ${b}/><rect x="14" y="4" width="4" height="16" rx="1" ${b}/>`],["pause-circle","Pausa en círculo","pausar",'<circle cx="12" cy="12" r="9"/><path d="M10 8v8M14 8v8"/>'],["stop","Parar","detener stop",`<rect x="5" y="5" width="14" height="14" rx="2" ${b}/>`],["record","Grabar","rec grabacion",`<circle cx="12" cy="12" r="7" ${b}/>`],["skip-next","Siguiente pista","next adelante",`<path d="M5 4.5l11 7.5-11 7.5z" ${b}/><rect x="17" y="4.5" width="3" height="15" rx="1" ${b}/>`],["skip-prev","Pista anterior","prev atras",`<path d="M19 4.5v15L8 12z" ${b}/><rect x="4" y="4.5" width="3" height="15" rx="1" ${b}/>`],["fast-forward","Avance rápido","ff acelerar",`<path d="M3 5l9 7-9 7z" ${b}/><path d="M13 5l9 7-9 7z" ${b}/>`],["rewind","Retroceso rápido","rw rebobinar",`<path d="M21 5l-9 7 9 7z" ${b}/><path d="M11 5l-9 7 9 7z" ${b}/>`],["volume-high","Volumen alto","sonido subir altavoz",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 8.5a5 5 0 0 1 0 7M19.5 6a9 9 0 0 1 0 12"/>`],["volume-low","Volumen bajo","sonido bajar altavoz",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 8.5a5 5 0 0 1 0 7"/>`],["volume-mute","Silencio","mute sonido apagado",`<path d="M4 9h4l5-4v14l-5-4H4z" ${b}/><path d="M16.5 9l5 6M21.5 9l-5 6"/>`],["mic","Micrófono","grabar voz",'<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/>'],["mic-off","Micrófono apagado","silenciar mute voz",'<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/><path d="M3 3l18 18"/>'],["headphones","Auriculares","cascos audio",'<path d="M4 15v-3a8 8 0 0 1 16 0v3"/><rect x="2" y="14" width="5" height="7" rx="2"/><rect x="17" y="14" width="5" height="7" rx="2"/>'],["music","Música","nota cancion",'<path d="M9 18V5l11-2v13"/><circle cx="6" cy="18" r="3"/><circle cx="17" cy="16" r="3"/>'],["playlist","Lista de reproducción","cola",'<path d="M3 6h12M3 11h12M3 16h7"/><circle cx="17" cy="17.5" r="2.5"/><path d="M19.5 17.5V9l2.5 1"/>'],["camera","Cámara","foto captura",'<path d="M3 9a2 2 0 0 1 2-2h2l1.5-2.5h7L17 7h2a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><circle cx="12" cy="13" r="3.5"/>'],["video","Vídeo","camara grabar",'<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/>'],["video-off","Vídeo apagado","camara cortar",'<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/><path d="M3 3l18 18"/>'],["image","Imagen","foto galeria",'<rect x="3" y="4" width="18" height="16" rx="2"/><circle cx="8.5" cy="9.5" r="1.8"/><path d="M4 18l5-5 3.5 3.5L16 13l4 4"/>'],["film","Película","cine video",'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M7 4v16M17 4v16M2 9h5M2 15h5M17 9h5M17 15h5"/>'],["tv","Televisión","pantalla monitor",'<rect x="2" y="6" width="20" height="13" rx="2"/><path d="M8 2l4 4 4-4"/>'],["cast","Enviar a pantalla","streaming airplay",'<path d="M2 16.5a5.5 5.5 0 0 1 5.5 5.5M2 12a10 10 0 0 1 10 10"/><path d="M2 8V6a2 2 0 0 1 2-2h16a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2h-5"/>'],["eject","Expulsar","sacar disco",`<path d="M12 4l8 10H4z" ${b}/><rect x="4" y="17" width="16" height="3" rx="1" ${b}/>`]],edit:[["copy","Copiar","duplicar ctrlc",'<rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>'],["paste","Pegar","ctrlv portapapeles",'<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>'],["scissors","Cortar","tijeras ctrlx",'<circle cx="6" cy="6" r="2.5"/><circle cx="6" cy="18" r="2.5"/><path d="M8 7.5L20 19M20 5L8 16.5"/>'],["undo","Deshacer","ctrlz atras",'<path d="M4 12h11a5 5 0 0 1 0 10h-3"/><path d="M8 8l-4 4 4 4"/>'],["redo","Rehacer","ctrly adelante",'<path d="M20 12H9a5 5 0 0 0 0 10h3"/><path d="M16 8l4 4-4 4"/>'],["save","Guardar","ctrls disquete",'<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>'],["trash","Borrar","papelera eliminar",'<path d="M3 6h18M8 6V4h8v2"/><path d="M6 6l1 14h10l1-14"/><path d="M10 10v7M14 10v7"/>'],["pencil","Lápiz","editar escribir",'<path d="M17 3l4 4L8 20l-5 1 1-5z"/><path d="M14.5 5.5l4 4"/>'],["brush","Pincel","pintar dibujar",'<path d="M9 14l-3 3c-1 1-1 3 0 4s3 1 4 0l3-3z"/><path d="M11 12l8-8 3 3-8 8z"/>'],["eraser","Borrador","goma",'<path d="M8.5 20.5l-5-5a2 2 0 0 1 0-2.8L13.6 2.6a2 2 0 0 1 2.8 0l4.5 4.5a2 2 0 0 1 0 2.8L11 20.5z"/><path d="M9 20.5h11M8.5 8l7 7"/>'],["text","Texto","letra fuente tipografia",'<path d="M4 7V4h16v3M12 4v16M8 20h8"/>'],["bold","Negrita","grueso",'<path d="M7 4h6a4 4 0 0 1 0 8H7zM7 12h7a4 4 0 0 1 0 8H7z"/>'],["italic","Cursiva","inclinada",'<path d="M10 4h8M6 20h8M14.5 4l-5 16"/>'],["underline","Subrayado","linea",'<path d="M7 4v7a5 5 0 0 0 10 0V4M5 20h14"/>'],["align-left","Alinear a la izquierda","parrafo",'<path d="M3 5h18M3 10h11M3 15h15M3 20h8"/>'],["align-center","Centrar","parrafo",'<path d="M3 5h18M6.5 10h11M4 15h16M8 20h8"/>'],["align-right","Alinear a la derecha","parrafo",'<path d="M3 5h18M10 10h11M6 15h15M13 20h8"/>'],["list-ul","Lista","viñetas puntos",`<path d="M9 6h12M9 12h12M9 18h12"/><circle cx="4.5" cy="6" r="1.4" ${b}/><circle cx="4.5" cy="12" r="1.4" ${b}/><circle cx="4.5" cy="18" r="1.4" ${b}/>`],["list-ol","Lista numerada","orden",'<path d="M9 6h12M9 12h12M9 18h12"/><path d="M3 4.5h1.5V9M3 10.5h2.5L3 14.5h2.5M3 16h2.5v2H3.5v2H6"/>'],["link","Enlace","url vinculo",'<path d="M10.5 13.5a4.5 4.5 0 0 0 6.4 0l2.5-2.5a4.5 4.5 0 0 0-6.4-6.4l-1.4 1.4"/><path d="M13.5 10.5a4.5 4.5 0 0 0-6.4 0l-2.5 2.5a4.5 4.5 0 0 0 6.4 6.4l1.4-1.4"/>'],["unlink","Quitar enlace","romper vinculo",'<path d="M9 15l-1.5 1.5a4.5 4.5 0 0 1-6.4-6.4L4 7.5"/><path d="M15 9l1.5-1.5a4.5 4.5 0 0 1 6.4 6.4L20 16.5"/><path d="M3 3l18 18"/>'],["crop","Recortar","encuadre",'<path d="M6 2v16h16M2 6h16v16"/>'],["layers","Capas","niveles",'<path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/>'],["palette","Paleta","color pintura",`<path d="M12 3a9 9 0 0 0 0 18c1.5 0 2-1 1.5-2s0-2 1.5-2H18a3 3 0 0 0 3-3c0-5-4-9-9-9z"/><circle cx="7.5" cy="12.5" r="1.3" ${b}/><circle cx="9.5" cy="8.5" r="1.3" ${b}/><circle cx="14" cy="7.5" r="1.3" ${b}/>`],["zoom-in","Acercar","lupa aumentar zoom",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5M10.5 8v5"/>'],["zoom-out","Alejar","lupa reducir zoom",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5"/>'],["grid","Rejilla","cuadricula tabla",'<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>'],["move","Mover","desplazar arrastrar",'<path d="M12 3v18M3 12h18"/><path d="M9 6l3-3 3 3M9 18l3 3 3-3M6 9l-3 3 3 3M18 9l3 3-3 3"/>'],["magic-wand","Varita mágica","automatico efecto",'<path d="M15 4.5l4.5 4.5L9 19.5 4.5 15z"/><path d="M12.5 7l4.5 4.5"/><path d="M19 2v3M22.5 4.5h-3M21 9l1.5.8"/>'],["pipette","Cuentagotas","color muestra",'<path d="M18 2l4 4-2.5 2.5-1-1L8 20l-4 1 1-4L16.5 5.5l-1-1z"/>']],system:[["power","Encendido","apagar boton power",'<path d="M12 3v9"/><path d="M7.5 6.5a7 7 0 1 0 9 0"/>'],["gear","Ajustes","configuracion opciones tuerca",'<path d="M10.3 3h3.4l.4 2.3 1.9.8 1.9-1.3 2.4 2.4-1.3 1.9.8 1.9 2.3.4v3.4l-2.3.4-.8 1.9 1.3 1.9-2.4 2.4-1.9-1.3-1.9.8-.4 2.3h-3.4l-.4-2.3-1.9-.8-1.9 1.3-2.4-2.4 1.3-1.9-.8-1.9L2.5 13.7v-3.4l2.3-.4.8-1.9-1.3-1.9 2.4-2.4 1.9 1.3 1.9-.8z"/><circle cx="12" cy="12" r="3"/>'],["sliders","Controles","ecualizador ajustes",'<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2.2"/><circle cx="15" cy="12" r="2.2"/><circle cx="8" cy="18" r="2.2"/>'],["home","Inicio","casa principal",'<path d="M3 11l9-7 9 7"/><path d="M5.5 9.5V20h13V9.5"/><path d="M10 20v-5h4v5"/>'],["search","Buscar","lupa encontrar",'<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21"/>'],["lock","Bloqueado","candado seguridad",'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/>'],["unlock","Desbloqueado","candado abierto",'<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 7.6-2"/>'],["key","Llave","clave contraseña",'<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>'],["user","Usuario","perfil persona cuenta",'<circle cx="12" cy="8" r="4"/><path d="M4 21c0-4.4 3.6-7 8-7s8 2.6 8 7"/>'],["users","Usuarios","grupo equipo",'<circle cx="9" cy="8" r="3.5"/><path d="M2 21c0-4 3.1-6.5 7-6.5s7 2.5 7 6.5"/><path d="M16 5.3a3.5 3.5 0 0 1 0 6.9M18 14.4c2.4.8 4 2.8 4 5.6"/>'],["bell","Aviso","notificacion campana alerta",'<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/>'],["bell-off","Avisos apagados","silenciar notificacion",'<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/><path d="M3 3l18 18"/>'],["wifi","Wi-Fi","red inalambrica",`<path d="M2.5 8.5a15 15 0 0 1 19 0"/><path d="M5.8 12.2a10 10 0 0 1 12.4 0"/><path d="M9 15.9a5 5 0 0 1 6 0"/><circle cx="12" cy="19.5" r="1.4" ${b}/>`],["wifi-off","Sin Wi-Fi","red caida",`<path d="M5.8 12.2a10 10 0 0 1 8-2.1"/><path d="M9 15.9a5 5 0 0 1 4-.8"/><circle cx="12" cy="19.5" r="1.4" ${b}/><path d="M3 3l18 18"/>`],["bluetooth","Bluetooth","emparejar",'<path d="M7 7l10 10-5 4V3l5 4L7 17"/>'],["battery","Batería","pila carga",`<rect x="2" y="7" width="17" height="10" rx="2.5"/><path d="M21.5 10.5v3"/><rect x="4.5" y="9.5" width="7" height="5" rx="1" ${b}/>`],["battery-charging","Cargando","bateria enchufe",'<path d="M14 7h3a2.5 2.5 0 0 1 2 2v6a2.5 2.5 0 0 1-2 2h-4"/><path d="M8 7H4.5a2.5 2.5 0 0 0-2.5 2.5v5A2.5 2.5 0 0 0 4.5 17H8"/><path d="M22 10.5v3"/><path d="M12 5.5L8.5 12H12l-1 6.5 4.5-7H12z"/>'],["usb","USB","conector cable",`<circle cx="12" cy="20.5" r="1.6" ${b}/><path d="M12 19V4"/><path d="M9 7l3-4 3 4"/><path d="M12 13l4-3V7.5"/><circle cx="16" cy="6.5" r="1.6" ${b}/><path d="M12 16l-4-3V10"/><rect x="6.5" y="7.5" width="3" height="2.5" ${b}/>`],["monitor","Monitor","pantalla ordenador",'<rect x="2" y="4" width="20" height="13" rx="2"/><path d="M8 21h8M12 17v4"/>'],["laptop","Portátil","ordenador",'<rect x="4" y="5" width="16" height="11" rx="2"/><path d="M2 19h20"/>'],["keyboard","Teclado","teclas escribir",'<rect x="2" y="6" width="20" height="12" rx="2"/><path d="M6 10h.01M10 10h.01M14 10h.01M18 10h.01M8 14h8"/>'],["mouse","Ratón","puntero clic",'<rect x="7" y="3" width="10" height="18" rx="5"/><path d="M12 7v3.5"/>'],["cpu","Procesador","chip micro",`<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9.5" y="9.5" width="5" height="5" rx="1" ${b}/><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3"/>`],["hdd","Disco","almacenamiento unidad",`<rect x="2" y="9" width="20" height="10" rx="2"/><path d="M5 4h14l3 5H2z"/><circle cx="17.5" cy="14" r="1.4" ${b}/>`],["printer","Impresora","imprimir",'<path d="M6 9V3h12v6"/><rect x="2" y="9" width="20" height="8" rx="2"/><path d="M6 14h12v7H6z"/>'],["calculator","Calculadora","numeros cuentas",'<rect x="4" y="2" width="16" height="20" rx="2"/><rect x="7" y="5" width="10" height="4" rx="1"/><path d="M8 13h.01M12 13h.01M16 13h.01M8 17.5h.01M12 17.5h.01M16 17.5h.01"/>'],["clock","Reloj","hora tiempo",'<circle cx="12" cy="12" r="9"/><path d="M12 6.5V12l3.5 2"/>'],["alarm","Alarma","despertador aviso",'<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4l3 2"/><path d="M5.5 3L2.5 6M18.5 3l3 3"/>'],["calendar","Calendario","fecha agenda",'<rect x="3" y="5" width="18" height="16" rx="2"/><path d="M3 10h18M8 3v4M16 3v4"/>'],["folder","Carpeta","directorio archivos",'<path d="M3 7a2 2 0 0 1 2-2h4l2 2.5h8a2 2 0 0 1 2 2V18a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>'],["folder-open","Carpeta abierta","directorio",'<path d="M3 8a2 2 0 0 1 2-2h4l2 2.5h6a2 2 0 0 1 2 2V12"/><path d="M3 20l2.6-7.3a1.5 1.5 0 0 1 1.4-1H20.5a1 1 0 0 1 .95 1.3L19 20z"/>'],["file","Archivo","documento fichero",'<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5"/>'],["file-text","Documento","texto informe",'<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5M9 13h6M9 17h4"/>'],["files","Archivos","documentos varios",'<rect x="8" y="3" width="12" height="15" rx="2"/><path d="M16 21H6a2 2 0 0 1-2-2V7"/>'],["download","Descargar","bajar guardar",'<path d="M12 3v12M7 11l5 5 5-5"/><path d="M4 20h16"/>'],["upload","Subir","enviar cargar",'<path d="M12 21V9M7 13l5-5 5 5"/><path d="M4 4h16"/>'],["sync","Sincronizar","actualizar refrescar",'<path d="M20 11a8 8 0 0 0-13.7-5.2L4 8"/><path d="M4 3.5V8h4.5"/><path d="M4 13a8 8 0 0 0 13.7 5.2L20 16"/><path d="M20 20.5V16h-4.5"/>'],["cloud","Nube","internet online",'<path d="M6.8 19a4.5 4.5 0 0 1-.4-9 6 6 0 0 1 11.4 1 4.2 4.2 0 0 1-.6 8z"/>'],["cloud-up","Subir a la nube","backup copia",'<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 21v-8M9 15.5l3-3 3 3"/>'],["cloud-down","Bajar de la nube","restaurar descarga",'<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 13v8M9 18.5l3 3 3-3"/>'],["database","Base de datos","datos sql",'<ellipse cx="12" cy="6" rx="8" ry="3"/><path d="M4 6v12c0 1.7 3.6 3 8 3s8-1.3 8-3V6"/><path d="M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3"/>'],["server","Servidor","rack hosting",'<rect x="3" y="4" width="18" height="7" rx="2"/><rect x="3" y="13" width="18" height="7" rx="2"/><path d="M7 7.5h.01M7 16.5h.01"/>'],["network","Red","nodos conexion",'<rect x="9" y="2" width="6" height="6" rx="1"/><rect x="2" y="16" width="6" height="6" rx="1"/><rect x="16" y="16" width="6" height="6" rx="1"/><path d="M12 8v3M5 16v-3h14v3"/>'],["shield","Escudo","seguridad proteccion",'<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/>'],["shield-check","Protegido","seguridad ok antivirus",'<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/><path d="M8.5 12l2.5 2.5 4.5-5"/>'],["plug","Enchufe","conectar corriente",'<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>'],["menu","Menú","hamburguesa opciones",'<path d="M4 7h16M4 12h16M4 17h16"/>'],["more-h","Más opciones","puntos horizontal",`<circle cx="5" cy="12" r="1.8" ${b}/><circle cx="12" cy="12" r="1.8" ${b}/><circle cx="19" cy="12" r="1.8" ${b}/>`],["more-v","Más opciones vertical","puntos",`<circle cx="12" cy="5" r="1.8" ${b}/><circle cx="12" cy="12" r="1.8" ${b}/><circle cx="12" cy="19" r="1.8" ${b}/>`],["filter","Filtrar","embudo criterios",'<path d="M3 5h18l-7 8v6l-4 2v-8z"/>'],["sort","Ordenar","clasificar",'<path d="M3 7h10M3 12h7M3 17h4"/><path d="M17 5v14M14 16l3 3 3-3"/>'],["eye","Ver","ojo mostrar visible",'<path d="M2 12s3.6-6.5 10-6.5S22 12 22 12s-3.6 6.5-10 6.5S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>'],["eye-off","Ocultar","ojo invisible",'<path d="M4.5 7.5C3 9.2 2 12 2 12s3.6 6.5 10 6.5c2 0 3.7-.5 5.1-1.3M9.5 5.8A11 11 0 0 1 12 5.5c6.4 0 10 6.5 10 6.5s-1 1.8-2.7 3.5"/><path d="M9.9 9.9a3 3 0 0 0 4.2 4.2"/><path d="M3 3l18 18"/>'],["maximize","Maximizar","pantalla completa",'<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/>'],["minimize","Minimizar","reducir ventana",'<path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5"/>'],["external-link","Abrir fuera","enlace externo ventana",'<path d="M14 4h6v6M20 4L10 14"/><path d="M18 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7a1 1 0 0 1 1-1h5"/>'],["window","Ventana","app programa",'<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 9h18M6.5 6.5h.01M9.5 6.5h.01"/>']],dev:[["code","Código","programar corchetes",'<path d="M9 18l-6-6 6-6M15 6l6 6-6 6"/>'],["braces","Llaves","json bloque",'<path d="M9 3c-2 0-3 1-3 3v2c0 2-1 3-2 3 1 0 2 1 2 3v2c0 2 1 3 3 3"/><path d="M15 3c2 0 3 1 3 3v2c0 2 1 3 2 3-1 0-2 1-2 3v2c0 2-1 3-3 3"/>'],["terminal","Terminal","consola shell cmd",'<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>'],["bug","Depurar","error bicho debug",'<rect x="8" y="7" width="8" height="12" rx="4"/><path d="M8 11H4M8 15.5H3.5M16 11h4M16 15.5h4.5M9.5 7.5L8 5M14.5 7.5L16 5M12 19v3"/>'],["git-branch","Rama","git branch",'<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="9" r="2.5"/><path d="M7 8.5v7M17 11.5c0 3.5-3.5 4-7 4.5"/>'],["git-commit","Commit","git guardar",'<circle cx="12" cy="12" r="3.2"/><path d="M2 12h6.8M15.2 12H22"/>'],["git-merge","Fusionar","git merge",'<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="13" r="2.5"/><path d="M7 8.5v7M14.5 13c-5 0-7.5-1.5-7.5-4.5"/>'],["package","Paquete","npm modulo caja",'<path d="M21 8l-9-5-9 5v8l9 5 9-5z"/><path d="M3 8l9 5 9-5M12 13v8"/>'],["box","Caja","contenedor almacen",'<rect x="3" y="7" width="18" height="13" rx="2"/><path d="M3 11h18M8 7V4h8v3"/>'],["robot","Robot","bot automatizar ia",`<rect x="4" y="8" width="16" height="12" rx="3"/><circle cx="9" cy="14" r="1.6" ${b}/><circle cx="15" cy="14" r="1.6" ${b}/><path d="M12 4v4M9 3.5h6"/>`],["activity","Actividad","pulso monitor rendimiento",'<path d="M3 12h4l3 8 4-16 3 8h4"/>'],["chart-bar","Gráfico de barras","estadisticas datos",'<path d="M3 21h18"/><path d="M6 21V11M12 21V4M18 21v-6"/>'],["chart-line","Gráfico de líneas","tendencia datos",'<path d="M3 3v18h18"/><path d="M6.5 15l4-4.5 3 3 6.5-7.5"/>'],["chart-pie","Gráfico circular","tarta porcentaje",'<path d="M12 3a9 9 0 1 0 9 9h-9z"/><path d="M14.5 2.3A9 9 0 0 1 21.7 9.5H14.5z"/>']],comm:[["mail","Correo","email sobre mensaje",'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M3 7l9 6 9-6"/>'],["mail-open","Correo abierto","email leido",'<path d="M3 10l9-6 9 6v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><path d="M3 10l9 6 9-6"/>'],["send","Enviar","mandar avion mensaje",'<path d="M21 3L10.5 13.5M21 3l-7 18-3.5-7.5L3 10z"/>'],["chat","Chat","mensaje conversacion",'<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/>'],["chat-dots","Mensaje","chat escribiendo",`<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/><circle cx="8" cy="10" r="1.2" ${b}/><circle cx="12" cy="10" r="1.2" ${b}/><circle cx="16" cy="10" r="1.2" ${b}/>`],["phone","Teléfono","llamar llamada",'<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/>'],["phone-off","Colgar","llamada cortar",'<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/><path d="M3 3l18 18"/>'],["at","Arroba","email usuario",'<circle cx="12" cy="12" r="4"/><path d="M16 8v5.5a2.5 2.5 0 0 0 5 0V12A9 9 0 1 0 17.4 19.2"/>'],["share","Compartir","enviar nodos",'<circle cx="18" cy="5" r="2.5"/><circle cx="6" cy="12" r="2.5"/><circle cx="18" cy="19" r="2.5"/><path d="M8.2 10.8l7.6-4.4M8.2 13.2l7.6 4.4"/>'],["megaphone","Megáfono","anuncio aviso",'<path d="M3 10.5v3l11 5V5.5z"/><path d="M14 9a3 3 0 0 1 0 6"/><path d="M6 15v4.5h3.5V16.5"/>'],["rss","RSS","feed suscripcion",`<circle cx="6" cy="18" r="1.9" ${b}/><path d="M4 11a9 9 0 0 1 9 9M4 4a16 16 0 0 1 16 16"/>`]],misc:[["flag","Bandera","marcar objetivo",'<path d="M5 21V4M5 5h13l-2.5 4.5L18 14H5"/>'],["pin","Marcador","ubicacion mapa chincheta",'<path d="M12 21.5S19 14.5 19 10a7 7 0 1 0-14 0c0 4.5 7 11.5 7 11.5z"/><circle cx="12" cy="10" r="2.5"/>'],["map","Mapa","ubicacion ruta",'<path d="M9 4L3 6.5v13L9 17l6 2.5 6-2.5v-13L15 6.5z"/><path d="M9 4v13M15 6.5v13"/>'],["globe","Mundo","internet web idioma",'<circle cx="12" cy="12" r="9"/><path d="M3 12h18"/><path d="M12 3a14 14 0 0 1 0 18 14 14 0 0 1 0-18z"/>'],["compass","Brújula","norte direccion",'<circle cx="12" cy="12" r="9"/><path d="M15.5 8.5l-2 5-5 2 2-5z"/>'],["sun","Sol","brillo claro dia",'<circle cx="12" cy="12" r="4"/><path d="M12 2v2.5M12 19.5V22M4.2 4.2l1.8 1.8M18 18l1.8 1.8M2 12h2.5M19.5 12H22M4.2 19.8L6 18M18 6l1.8-1.8"/>'],["moon","Luna","noche oscuro dormir",'<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>'],["flame","Fuego","llama caliente racha",'<path d="M12 2.5s5.5 4.8 5.5 9.5a5.5 5.5 0 0 1-11 0c0-2.2 1-3.9 2.2-5 0 2.2 1 3.2 2 3.2 1.6 0 1.1-4.3-1.2-8z"/>'],["drop","Gota","agua humedad",'<path d="M12 3.5s6 6.4 6 10.5a6 6 0 0 1-12 0C6 9.9 12 3.5 12 3.5z"/>'],["leaf","Hoja","planta eco natural",'<path d="M4 20c0-9 6-14 16-14 0 9-5 14-12 14H4z"/><path d="M9 15c2-3 5-5 8-6"/>'],["bolt","Rayo","energia rapido",'<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>'],["rocket","Cohete","lanzar rapido inicio",'<path d="M12 2c3.5 2.6 5.5 6.6 5.5 11l-2.5 3h-6l-2.5-3C6.5 8.6 8.5 4.6 12 2z"/><circle cx="12" cy="9.5" r="2"/><path d="M9 19l-2.5 3M15 19l2.5 3"/>'],["gift","Regalo","premio caja",'<rect x="3" y="8" width="18" height="4" rx="1"/><path d="M5 12v9h14v-9M12 8v13"/><path d="M12 8S10.5 4 8.5 4a2 2 0 0 0 0 4zM12 8s1.5-4 3.5-4a2 2 0 0 1 0 4z"/>'],["cart","Carrito","compra tienda",`<circle cx="9.5" cy="20" r="1.7" ${b}/><circle cx="18" cy="20" r="1.7" ${b}/><path d="M2 3h3l2.5 12h12L22 6.5H6"/>`],["tag","Etiqueta","precio marcar",`<path d="M11 3H4a1 1 0 0 0-1 1v7l10 10 8-8z"/><circle cx="7.5" cy="7.5" r="1.5" ${b}/>`],["wallet","Cartera","dinero pagos",`<rect x="3" y="6" width="18" height="14" rx="2"/><path d="M3 10h18"/><circle cx="17" cy="15" r="1.4" ${b}/>`],["credit-card","Tarjeta","pago banco",'<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M2 10h20M6 15h4"/>'],["coffee","Café","descanso taza",'<path d="M3 8h14v6a5 5 0 0 1-10 0z"/><path d="M17 9h2a2.5 2.5 0 0 1 0 5h-2"/><path d="M3 21h16"/>'],["gamepad","Mando","juego consola",`<rect x="2" y="7" width="20" height="11" rx="4"/><path d="M7 10.5v4M5 12.5h4"/><circle cx="16" cy="11.5" r="1.3" ${b}/><circle cx="18.5" cy="14" r="1.3" ${b}/>`],["dice","Dado","azar juego",`<rect x="3" y="3" width="18" height="18" rx="3"/><circle cx="8" cy="8" r="1.5" ${b}/><circle cx="12" cy="12" r="1.5" ${b}/><circle cx="16" cy="16" r="1.5" ${b}/>`],["trophy","Trofeo","premio ganar",'<path d="M8 3h8v6a4 4 0 0 1-8 0z"/><path d="M8 5H5v1.5A3.5 3.5 0 0 0 8.5 10M16 5h3v1.5A3.5 3.5 0 0 1 15.5 10"/><path d="M12 13v4M8.5 21h7l-1-4h-5z"/>'],["target","Objetivo","diana precision",`<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1.6" ${b}/>`],["timer","Temporizador","cronometro tiempo",'<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4M9.5 2h5"/>'],["hourglass","Reloj de arena","espera tiempo",'<path d="M6 2h12M6 22h12"/><path d="M8 2v4l4 4 4-4V2M8 22v-4l4-4 4 4v4"/>'],["thermometer","Temperatura","calor frio",'<path d="M14 14.2V5a2 2 0 0 0-4 0v9.2a4 4 0 1 0 4 0z"/>'],["wrench","Llave inglesa","herramienta arreglar",'<path d="M20 5.5A5.5 5.5 0 0 1 12.6 13L5 20.6 3.4 19 11 11.4A5.5 5.5 0 0 1 18.5 4L15 7.5l1.5 1.5L20 5.5z"/>'],["hammer","Martillo","herramienta construir",'<path d="M14 3l7 7-3 3-7-7z"/><path d="M11.5 8.5L3 17v4h4l8.5-8.5"/>'],["toolbox","Herramientas","caja utiles",'<rect x="2" y="8" width="20" height="12" rx="2"/><path d="M8 8V5h8v3M2 13h20"/>'],["lightbulb","Idea","bombilla consejo",'<path d="M9 18.5h6M10 21.5h4"/><path d="M12 2.5a6 6 0 0 0-3.5 10.9v2.1h7v-2.1A6 6 0 0 0 12 2.5z"/>'],["magnet","Imán","atraer",'<path d="M6 3H3v9a9 9 0 0 0 18 0V3h-3v9a6 6 0 0 1-12 0z"/><path d="M3 8h3M18 8h3"/>'],["anchor","Ancla","fijar barco",'<circle cx="12" cy="5" r="2.5"/><path d="M12 7.5V21"/><path d="M5 12H3a9 9 0 0 0 18 0h-2"/><path d="M8 11h8"/>'],["plane","Avión","viaje vuelo",'<path d="M10 3.5a2 2 0 0 1 4 0V9l8 4.5v2.5l-8-2.5v4l2.5 2v2L12 20l-4.5 1.5v-2l2.5-2v-4L2 16v-2.5L10 9z"/>'],["car","Coche","vehiculo transporte",`<path d="M3 16v-3.5L5 7h14l2 5.5V16z"/><path d="M4.5 16v3H7v-3M17 16v3h2.5v-3"/><circle cx="7.5" cy="12.5" r="1.3" ${b}/><circle cx="16.5" cy="12.5" r="1.3" ${b}/>`],["book","Libro","leer manual",'<path d="M4 4.5A2.5 2.5 0 0 1 6.5 2H20v16H6.5A2.5 2.5 0 0 0 4 20.5z"/><path d="M4 20.5A2.5 2.5 0 0 1 6.5 18H20v4H6.5A2.5 2.5 0 0 1 4 20.5z"/>'],["bookmark","Marcador","guardar favorito",'<path d="M6 3h12v18l-6-4.5L6 21z"/>'],["graduation","Formación","estudios birrete",'<path d="M2 8l10-4 10 4-10 4z"/><path d="M6 10.5V16c0 1.7 2.7 3 6 3s6-1.3 6-3v-5.5"/>'],["thumbs-up","Me gusta","aprobar bien",'<path d="M7 21V10l5-8a2.5 2.5 0 0 1 2.4 3.2L13.5 9H20a2 2 0 0 1 2 2.4l-1.6 7.2A2 2 0 0 1 18.4 21z"/><rect x="2" y="10" width="5" height="11" rx="1"/>'],["smiley","Sonrisa","cara emoji feliz",`<circle cx="12" cy="12" r="9"/><circle cx="9" cy="10" r="1.3" ${b}/><circle cx="15" cy="10" r="1.3" ${b}/><path d="M7.8 14.3a5 5 0 0 0 8.4 0"/>`],["skull","Calavera","peligro muerte",`<path d="M5 11.5a7 7 0 1 1 14 0V14l-2 1.5V19h-3v-2h-4v2H7v-3.5L5 14z"/><circle cx="9.3" cy="11.5" r="1.8" ${b}/><circle cx="14.7" cy="11.5" r="1.8" ${b}/>`],["crown","Corona","rey premium",'<path d="M3 8l4.5 4L12 4.5 16.5 12 21 8v11H3z"/>']]},cr=Object.entries(Gd).flatMap(([e,t])=>t.map(([a,n,r,i])=>({id:a,name:n,cat:e,keywords:r,body:i})));function Xr(e){return String(e).toLowerCase().normalize("NFD").replace(/[̀-ͯ]/g,"")}function Kd(e="",t="all"){const a=Xr(e).split(/\s+/).filter(Boolean);return cr.filter(n=>{if(t!=="all"&&n.cat!==t)return!1;if(!a.length)return!0;const r=Xr(`${n.id} ${n.name} ${n.keywords}`);return a.every(i=>r.includes(i))})}function No(e,{size:t=24,color:a=null,strokeWidth:n=2}={}){const r=a?` style="color:${a}"`:"";return`<svg xmlns="http://www.w3.org/2000/svg" width="${t}" height="${t}" viewBox="0 0 24 24"${r}
    fill="none" stroke="currentColor" stroke-width="${n}"
    stroke-linecap="round" stroke-linejoin="round">${e.body}</svg>`}function Xd(e){return cr.find(t=>t.id===e)||null}const Yd=3,Qd=28,Oo=9,qo=[{css:"Segoe UI",label:"Segoe UI"},{css:"Segoe UI Black",label:"Segoe UI Black"},{css:"Arial",label:"Arial"},{css:"Arial Black",label:"Arial Black"},{css:"Impact",label:"Impact (estrecha)"},{css:"Tahoma",label:"Tahoma"},{css:"Verdana",label:"Verdana"},{css:"Consolas",label:"Consolas (mono)"},{css:"Courier New",label:"Courier New (mono)"},{css:"Georgia",label:"Georgia"},{css:"Times New Roman",label:"Times New Roman"}],Zd=[{id:"pencil",icon:"pencil",label:"Lápiz"},{id:"eraser",icon:"eraser",label:"Borrador"},{id:"fill",icon:"fill",label:"Relleno"},{id:"select",icon:"select",label:"Selección (mover/redimensionar)"}],l={profile:0,kind:"key",page:0,key:0,layer:"normal",tool:"pencil",zoom:Oo,buffer:Ut(),drawing:!1,drawValue:1,lastPos:null,undoStack:[],dirty:!1,source:null,layerXf:null,dragging:null,selecting:null,outline:null,textDraft:"",textFont:"Segoe UI",textSize:16,textBold:!0,libQuery:"",libCat:"all"};function _o(){Ie();const e=document.getElementById("view-oled");e.addEventListener("click",tu),e.addEventListener("change",au),e.addEventListener("input",nu),window.addEventListener("keydown",ru),W("connected",()=>{vn()});let t=0;rt(()=>{s.profiles.length!==t&&(t=s.profiles.length,l.profile>=s.profiles.length&&(l.profile=0),vn(),Ie())})}function zo({profile:e,key:t,layer:a,kind:n}={}){Number.isInteger(e)&&e<s.profiles.length&&(l.profile=e),(a==="normal"||a==="super")&&(l.layer=a),l.kind=n==="profile"?"profile":"key",qa(!1),Number.isInteger(t)&&dr(t)&&(l.key=t),l.page=l.kind==="profile"?0:s.profiles[l.profile]?.pageIdx||0;const r=ye(l.profile,ie(),l.page);l.buffer=r?Uint8Array.from(r):Ut(),l.undoStack=[],l.dirty=!1,Ie(),vn()}function Bo(){qa(!1),nn("view-profiles")}function Jd(){l.dirty&&!confirm(`Se perderá lo que hayas dibujado en este icono.

¿Salir sin guardar?`)||Bo()}function dr(e){return _e[e]}function eu(e,t=l.layer){const a=dr(e);return a?a-1+(t==="super"?10:0):-1}function ie(){return l.kind==="profile"?xa:eu(l.key)}function vn(){je(l.profile)}function tu(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;a==="tool"?(l.tool=t.dataset.tool,wu()):a==="zoom-in"?oa(l.zoom+$n(l.zoom)):a==="zoom-out"?oa(l.zoom-$n(l.zoom-1)):a==="zoom-fit"?oa(iu()):a==="exit"?Jd():a==="clear"?(Xe(),wi(l.buffer),O()):a==="invert"?(Xe(),qs(l.buffer),O()):a==="frame"?(Xe(),Mi(l.buffer),O()):a==="undo"?gu():a==="make-text"?hu():a==="import"?document.getElementById("oled-file").click():a==="apply"?Go():a==="cancel"?qa(!0):a==="center"?Wo():a==="fit-layer"?du():a==="nudge"?cu(Number(t.dataset.dx),Number(t.dataset.dy)):a==="size-step"?pr(ur()+Number(t.dataset.d)):a==="xf-invert"?(l.layerXf.invert=t.dataset.mode,t.parentElement.querySelectorAll('[data-act="xf-invert"]').forEach(n=>n.classList.toggle("on",n.dataset.mode===t.dataset.mode)),O()):a==="lib-cat"?pu(t.dataset.cat):a==="lib-pick"?fu(t.dataset.id):a==="upload"?bu():a==="reset-slot"?vu():a==="load-current"&&$u()}function au(e){e.target.id==="oled-file"&&e.target.files?.[0]?(uu(e.target.files[0]),e.target.value=""):e.target.dataset.act==="text-font"?(l.textFont=e.target.value,l.source?.kind==="text"&&xn()):e.target.dataset.act==="layer-mode"&&(l.layerXf.mode=e.target.value,O())}function nu(e){const t=e.target.dataset.act;if(t)if(t==="lib-search")l.libQuery=e.target.value,Uo();else if(t==="text-draft")l.textDraft=e.target.value;else if(t==="text-size")l.textSize=Number(e.target.value),l.source?.kind==="text"&&xn();else if(t==="text-bold")l.textBold=e.target.checked,l.source?.kind==="text"&&xn();else if(l.layerXf)t==="xf-size"?pr(Number(e.target.value),!1):t==="xf-threshold"?(l.layerXf.threshold=Number(e.target.value),O(),Se()):t==="xf-blur"?(l.layerXf.blur=Number(e.target.value),O(),Se()):t==="xf-dither"&&(l.layerXf.dither=e.target.checked,O());else return}function ru(e){if(!l.layerXf)return;const t=["INPUT","TEXTAREA","SELECT"].includes(e.target.tagName);if(e.key==="Escape"){qa(!0);return}if(e.key==="Enter"&&!t){e.preventDefault(),Go();return}if(t)return;const a=e.shiftKey?5:1,n={ArrowLeft:[-a,0],ArrowRight:[a,0],ArrowUp:[0,-a],ArrowDown:[0,a]}[e.key];n&&(e.preventDefault(),l.layerXf.x+=n[0],l.layerXf.y+=n[1],O(),Se())}function $n(e){return e<8?1:e<16?2:4}function iu(){const e=document.getElementById("oled-canvas-wrap");if(!e)return Oo;const t=getComputedStyle(e),a=e.clientWidth-parseFloat(t.paddingLeft)-parseFloat(t.paddingRight),n=e.clientHeight-parseFloat(t.paddingTop)-parseFloat(t.paddingBottom);return jo(Math.min(a/w,n/I))}function jo(e){return Math.max(Yd,Math.min(Qd,Math.round(e)))}function oa(e,t=null){const a=jo(e);if(a===l.zoom)return;const n=document.getElementById("oled-canvas"),r=document.getElementById("oled-canvas-wrap");let i=null,o=null;if(t&&n){const c=n.getBoundingClientRect();i=(t.clientX-c.left)/l.zoom,o=(t.clientY-c.top)/l.zoom}if(l.zoom=a,O(),ou(),i!==null&&r&&n){const c=n.getBoundingClientRect();r.scrollLeft+=c.left+i*a-t.clientX,r.scrollTop+=c.top+o*a-t.clientY}}function ou(){const e=document.getElementById("oled-zoom-level");e&&(e.textContent=`${l.zoom}×`)}const Yr=4;function su(){const e=document.getElementById("oled-canvas"),t=document.getElementById("oled-canvas-wrap");if(!e)return;const a=r=>{const i=e.getBoundingClientRect();return{x:(r.clientX-i.left)/i.width*w,y:(r.clientY-i.top)/i.height*I}};t?.addEventListener("wheel",r=>{if(!r.ctrlKey)return;r.preventDefault();const i=r.deltaY<0?1:-1;oa(l.zoom+i*$n(i>0?l.zoom:l.zoom-1),r)},{passive:!1}),e.addEventListener("pointerdown",r=>{r.preventDefault(),e.setPointerCapture(r.pointerId);const i=a(r);if(l.layerXf){const u=wa(l.source,l.layerXf),p=l.outline||u,m=Math.abs(i.x-(p.x+p.width))<=Yr,v=Math.abs(i.y-(p.y+p.height))<=Yr;l.dragging=m&&v?{kind:"resize",originX:u.x,originY:u.y,startH:u.height,startW:u.width}:{kind:"move",grabX:i.x-u.x,grabY:i.y-u.y};return}if(l.tool==="select"){l.selecting={x0:i.x,y0:i.y,x1:i.x,y1:i.y};return}Xe();const o=Math.floor(i.x),c=Math.floor(i.y);if(l.tool==="fill"){_s(l.buffer,o,c,r.button===2?0:1),O();return}l.drawing=!0,l.drawValue=l.tool==="eraser"||r.button===2?0:1,l.lastPos={x:o,y:c},ne(l.buffer,o,c,l.drawValue),O()}),e.addEventListener("pointermove",r=>{const i=a(r);if(l.dragging){if(l.dragging.kind==="move")l.layerXf.x=i.x-l.dragging.grabX,l.layerXf.y=i.y-l.dragging.grabY,O(),Se();else{const{originX:u,originY:p,startW:m,startH:v}=l.dragging,y=Math.max((i.x-u)/Math.max(m,.001),(i.y-p)/Math.max(v,.001));pr(v*Math.max(.05,y))}return}if(l.selecting){l.selecting.x1=i.x,l.selecting.y1=i.y,O();return}if(!l.drawing)return;const o=Math.floor(i.x),c=Math.floor(i.y);l.lastPos&&zs(l.buffer,l.lastPos.x,l.lastPos.y,o,c,l.drawValue),l.lastPos={x:o,y:c},O()});const n=()=>{l.drawing=!1,l.lastPos=null,l.dragging=null,l.selecting&&mu()};e.addEventListener("pointerup",n),e.addEventListener("pointercancel",()=>{l.drawing=!1,l.lastPos=null,l.dragging=null,l.selecting=null}),e.addEventListener("contextmenu",r=>r.preventDefault()),e.style.cursor=l.layerXf?"move":"crosshair"}function lu(e){const t=jn(e),a=it(e);return{x:(w-a.width*t)/2-a.x*t,y:(I-a.height*t)/2-a.y*t,scale:t,threshold:128,blur:e.kind==="image"&&!e.crisp?1:0,dither:!1,invert:"none",mode:e.kind==="text"?"merge":"replace"}}const Do=2,Ho=I*3;function Vo(){return Math.max(1,it(l.source).height)}function ur(){return l.layerXf.scale*Vo()}function pr(e,t=!0){if(!l.layerXf)return;const a=Math.max(Do,Math.min(Ho,e)),n=wa(l.source,l.layerXf),r=it(l.source);l.layerXf.scale=a/Vo(),l.layerXf.x=n.x-r.x*l.layerXf.scale,l.layerXf.y=n.y-r.y*l.layerXf.scale,t&&Fo(),O(),Se()}function Fo(){const e=document.getElementById("xf-size");e&&(e.value=Math.round(ur()))}function cu(e,t){l.layerXf&&(l.layerXf.x+=e,l.layerXf.y+=t,O(),Se())}function Oa(e=null){return Gs(e||Gt(l.source,l.layerXf))||wa(l.source,l.layerXf)}function fr(){const e=it(l.source),t=l.layerXf.scale;l.layerXf.x=Math.round((w-e.width*t)/2)-e.x*t,l.layerXf.y=Math.round((I-e.height*t)/2)-e.y*t;const a=Oa();l.layerXf.x+=Math.round((w-a.width)/2-a.x),l.layerXf.y+=Math.round((I-a.height)/2-a.y)}function Wo(){fr(),O(),Se()}const Qr=1;function du(){l.layerXf.scale=jn(l.source);for(let e=0;e<6;e++){fr();const t=Oa(),a=Math.min((w-2*Qr)/t.width,(I-2*Qr)/t.height);if(Math.abs(a-1)<.02)break;l.layerXf.scale*=Math.max(.5,Math.min(2,a))}Fo(),Wo()}function hr(e){l.source=e,l.layerXf=lu(e),fr(),Ie()}async function uu(e){try{hr(await Ds(e))}catch(t){g(`No se pudo leer la imagen: ${t.message}`,"error")}}function pu(e){l.libCat=e,document.querySelectorAll('[data-act="lib-cat"]').forEach(t=>t.classList.toggle("on",t.dataset.cat===e)),Uo()}function Uo(){const e=document.getElementById("oled-lib-grid");e&&(e.outerHTML=Xo())}async function fu(e){const t=Xd(e);if(t)try{hr(await ki(No(t,{size:256,color:"#fff"})))}catch(a){g(`No se pudo cargar el icono: ${a.message}`,"error")}}function hu(){if(!l.textDraft.trim()){g("Escribe algo primero","error");return}hr(qn(l.textDraft,{fontSize:l.textSize,bold:l.textBold,font:l.textFont}))}function xn(){const e={...l.layerXf};l.source=qn(l.textDraft,{fontSize:l.textSize,bold:l.textBold,font:l.textFont}),l.layerXf=e,O(),Se()}function mu(){const{x0:e,y0:t,x1:a,y1:n}=l.selecting;l.selecting=null;const r=Math.max(0,Math.floor(Math.min(e,a))),i=Math.max(0,Math.floor(Math.min(t,n))),o=Math.min(w,Math.ceil(Math.max(e,a))),c=Math.min(I,Math.ceil(Math.max(t,n))),u=o-r,p=c-i;if(u<1||p<1){O();return}let m=!1;for(let h=i;h<c&&!m;h++)for(let k=r;k<o;k++)if(ht(l.buffer,k,h)){m=!0;break}if(!m){O();return}Xe();const v=document.createElement("canvas");v.width=u,v.height=p;const y=v.getContext("2d");y.fillStyle="#fff";for(let h=0;h<p;h++)for(let k=0;k<u;k++)ht(l.buffer,r+k,i+h)&&(y.fillRect(k,h,1,1),ne(l.buffer,r+k,i+h,0));l.source={kind:"image",bitmap:v,naturalWidth:u,naturalHeight:p,cut:!0},l.layerXf={x:r,y:i,scale:1,threshold:128,blur:0,dither:!1,invert:"none",mode:"merge"},Ie()}function Go(){if(!l.layerXf)return;Xe();const e=Gt(l.source,l.layerXf);l.buffer=Dn(l.buffer,e,l.layerXf.mode),l.source=null,l.layerXf=null,Ie()}function qa(e=!0){l.selecting=null,l.layerXf&&(l.source?.cut&&(l.buffer=l.undoStack.pop()||l.buffer),l.source=null,l.layerXf=null,e&&Ie())}function Xe(){l.undoStack.push(l.buffer.slice()),l.undoStack.length>30&&l.undoStack.shift(),l.dirty=!0}function gu(){const e=l.undoStack.pop();e&&(l.buffer=e,O())}function yu(){const{x0:e,y0:t,x1:a,y1:n}=l.selecting;return{x:Math.min(e,a),y:Math.min(t,n),width:Math.abs(a-e),height:Math.abs(n-t)}}function O(){let e=l.buffer;if(l.layerXf){const r=Gt(l.source,l.layerXf);e=Dn(l.buffer,r,l.layerXf.mode),l.outline=Oa(r)}else l.outline=l.selecting?yu():null;const t=l.outline,a=document.getElementById("oled-canvas");a&&tn(e,a,{zoom:l.zoom,outline:t});const n=document.getElementById("oled-preview");n&&tn(e,n,{zoom:2,grid:!1})}function Se(){if(!l.layerXf)return;const e=(a,n)=>{const r=document.getElementById(a);r&&(r.textContent=n)},t=l.outline||Oa();e("xf-size-val",`${Math.round(t.width)} × ${Math.round(t.height)} px`),e("xf-threshold-val",l.layerXf.threshold),e("xf-blur-val",l.layerXf.blur),e("xf-pos-val",`x ${Math.round(t.x)} · y ${Math.round(t.y)}`)}async function bu(){if(!Y())return;if(ie()<0){g("Esa tecla no tiene pantalla","error");return}const e=document.getElementById("btn-oled-upload");e&&(e.disabled=!0);try{l.kind!=="profile"&&await Qe(l.profile,l.page),await ve(l.profile,ie(),l.buffer),ze(l.profile,ie(),Uint8Array.from(l.buffer),l.page);const t=s.profiles[l.profile];t&&(l.kind==="profile"?t.pages[0].oledMask|=1<<ie():t.oledMask|=1<<ie()),z(),l.dirty=!1,g(l.kind==="profile"?"Icono enviado al perfil":`Icono enviado a la tecla ${l.key+1}`),Bo()}catch(t){g(`Error al enviar: ${t.message}`,"error"),e&&(e.disabled=!1)}}async function vu(){if(Y())try{l.kind!=="profile"&&await Qe(l.profile,l.page),await Ft(l.profile,ie()),ze(l.profile,ie(),null,l.page);const e=s.profiles[l.profile];e&&(l.kind==="profile"?e.pages[0].oledMask&=~(1<<ie()):e.oledMask&=~(1<<ie())),wi(l.buffer),z(),l.dirty=!1,Ie(),g(l.kind==="profile"?"Icono del perfil eliminado":"Icono eliminado; vuelve la etiqueta de texto")}catch(e){g(`Error: ${e.message}`,"error")}}async function $u(){if(!Y())return;l.kind!=="profile"&&await Qe(l.profile,l.page);const e=await An(l.profile,ie());if(!e){g(l.kind==="profile"?"Este perfil no tiene icono guardado":"Esa tecla no tiene icono guardado","info");return}ze(l.profile,ie(),e,l.page),Xe(),l.buffer=Uint8Array.from(e),O(),g("Icono cargado desde el teclado")}function Ie(){const e=document.getElementById("view-oled");e&&(e.innerHTML=`
    ${xu()}

    <div class="oled-grid">
      <div class="glass-panel oled-canvas-card">
        <div class="oled-toolbar" id="oled-toolbar">${Ko()}</div>
        <div class="oled-canvas-wrap" id="oled-canvas-wrap">
          <canvas id="oled-canvas"></canvas>
        </div>
        <p class="oled-hint">${Mu()}</p>
      </div>

      <div class="oled-side">
        ${l.layerXf?Pu():Eu()}
        ${Cu()}
      </div>
    </div>`,su(),O(),Se())}function xu(){const e=s.profiles[l.profile];if(l.kind==="profile")return`
      <header class="oled-actionbar">
        <div class="oled-actionbar-info">
          <h1>Icono del perfil</h1>
          <p>${jt(e?.name||`Perfil ${l.profile+1}`)}</p>
        </div>
        <div class="oled-actionbar-btns">
          <button class="secondary-btn" data-act="exit">${f("close",16)} Salir sin guardar</button>
          <button class="primary-btn" id="btn-oled-upload" data-act="upload">
            ${f("check",16)} Enviar al perfil
          </button>
        </div>
      </header>`;const t=nt()&&e&&D(e)>1?` · página ${l.page+1} de ${D(e)}`:"",a=`${jt(e?.name||`Perfil ${l.profile+1}`)} · tecla ${l.key+1} · pantalla ${dr(l.key)||"—"} · capa ${l.layer==="super"?"SUPER":"NORMAL"}${t}`;return`
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
    </header>`}function Ko(){const e=l.layerXf?"disabled":"";return`
    ${Zd.map(t=>`
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
    </span>`}function wu(){const e=document.getElementById("oled-toolbar");e&&(e.innerHTML=Ko())}function Mu(){return l.layerXf?`Arrastra para mover la capa, y tira de la esquina inferior derecha para escalarla.
            La cruceta y las <strong>flechas</strong> del teclado mueven 1 px (con Shift, 5).
            <strong>Enter</strong> fija, <strong>Esc</strong> cancela.`:`Clic izquierdo pinta, clic derecho borra. <strong>Ctrl + rueda</strong> hace zoom sobre el punto
          del cursor. Las líneas violetas marcan las páginas de 8 px en las que el SSD1306 direcciona la memoria.`}function Xo(){const e=Kd(l.libQuery,l.libCat);return e.length?`
    <div class="icon-lib-grid" id="oled-lib-grid">
      ${e.map(t=>`
        <button class="icon-lib-item" data-act="lib-pick" data-id="${t.id}" title="${jt(t.name)}">
          ${No(t,{size:28})}
        </button>`).join("")}
    </div>`:'<div class="icon-lib-grid empty" id="oled-lib-grid">Ningún icono coincide con la búsqueda.</div>'}function ku(){const e=cr.length;return`
    <div class="glass-panel oled-card">
      <div class="card-header">${f("square",20)}<h2>Biblioteca de iconos</h2></div>
      <p class="setting-desc mb-8">${e} iconos listos para usar. Pincha en uno y colócalo como quieras.</p>

      <input type="search" class="text-input" data-act="lib-search" placeholder="Buscar: guardar, volumen, carpeta…"
             value="${jt(l.libQuery)}">

      <div class="chip-row icon-lib-cats">
        <button class="chip ${l.libCat==="all"?"on":""}" data-act="lib-cat" data-cat="all">Todos</button>
        ${Ud.map(t=>`
          <button class="chip ${l.libCat===t.id?"on":""}" data-act="lib-cat" data-cat="${t.id}">${t.label}</button>`).join("")}
      </div>

      ${Xo()}
    </div>`}function Eu(){return`
    ${ku()}
    <div class="glass-panel oled-card">
      <div class="card-header">${f("text",20)}<h2>Generar contenido</h2></div>

      <label class="field">
        <span class="field-label">Texto</span>
        <textarea class="text-input" rows="2" data-act="text-draft"
                  placeholder="Enter = salto de línea">${jt(l.textDraft)}</textarea>
      </label>

      <label class="field">
        <span class="field-label">Fuente</span>
        <select class="select-input" data-act="text-font">
          ${qo.map(e=>`
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
    </div>`}function Pu(){const e=l.layerXf,t=l.source.kind==="text";return`
    <div class="glass-panel oled-card is-placing">
      <div class="card-header">${f("fit",20)}<h2>Colocando ${t?"texto":"imagen"}</h2></div>

      ${t?`
        <label class="field">
          <span class="field-label">Fuente</span>
          <select class="select-input" data-act="text-font">
            ${qo.map(a=>`
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
                 min="${Do}" max="${Ho}" step="1" value="${Math.round(ur())}">
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
    </div>`}function Cu(){return`
    <div class="glass-panel oled-card">
      <div class="card-header">${f("oled",20)}<h2>Pantalla</h2></div>
      <div class="oled-preview-box">
        <span class="field-label">Tamaño real</span>
        <canvas id="oled-preview" width="${w*2}" height="${I*2}"></canvas>
      </div>
      <div class="row-inline">
        <button class="secondary-btn" data-act="load-current">${f("refresh",16)} Releer del teclado</button>
        <button class="secondary-btn danger" data-act="reset-slot">${f("trash",16)} Quitar icono</button>
      </div>
      <p class="setting-desc">Recuerda pulsar <strong>Guardar en Flash</strong> para que sobreviva a la desconexión.</p>
    </div>`}function jt(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}const Su=Object.freeze(Object.defineProperty({__proto__:null,init:_o,openTarget:zo,render:Ie},Symbol.toStringTag,{value:"Module"})),$={available:!1,enabled:!1,rules:[],fallback:null,current:null,lastApplied:null,error:null},Iu=[{match:"photoshop",label:"Photoshop"},{match:"altium",label:"Altium Designer"},{match:"premiere",label:"Premiere Pro"},{match:"code",label:"VS Code"},{match:"excel",label:"Excel"},{match:"winword",label:"Word"},{match:"chrome",label:"Chrome"},{match:"blender",label:"Blender"}];async function Yo(){const e=document.getElementById("view-auto");e.addEventListener("click",Tu),e.addEventListener("change",Ru),$.available=await window.orby.foreground.available();const t=await window.orby.getConfig();Object.assign($,{enabled:t.autoProfile.enabled,rules:t.autoProfile.rules||[],fallback:t.autoProfile.fallback}),sn($.enabled),Ai(()=>ct()),window.orby.foreground.onChange(n=>{$.current=n,wn(n),ct()}),window.orby.foreground.onError(n=>{$.error=n,ct()}),W("connected",()=>setTimeout(()=>wn($.current),1500));let a=0;rt(()=>{s.profiles.length!==a?(a=s.profiles.length,Au(),Ce()):ct()}),$.enabled&&($.current=await window.orby.foreground.current()),Ce()}function Lu(e,t){return e?t==="title"?(e.title||"").toLowerCase():t==="process"?(e.process||"").toLowerCase():`${e.process||""} ${e.title||""}`.toLowerCase():""}function Qo(e){for(const t of $.rules){const a=(t.match||"").trim().toLowerCase();if(a&&Lu(e,t.field).includes(a))return t}return null}function Au(){const e=s.profiles.length-1;if(e<0)return;let t=!1;for(const a of $.rules)a.profile>e&&(a.profile=e,t=!0);$.fallback!==null&&$.fallback!==void 0&&$.fallback>e&&($.fallback=e,t=!0),t&&Oe()}async function wn(e){if(!$.enabled||!s.connected)return;$l(e);const t=Qo(e),a=t?t.profile:$.fallback;try{a!=null&&a!==s.activeProfileIdx&&(await si(a),s.activeProfileIdx=a,te()),$.lastApplied=t?.id??"fallback",await Oi(),ct()}catch{}}function Oe(){return window.orby.setConfig({autoProfile:{enabled:$.enabled,rules:$.rules,fallback:$.fallback}})}async function Tu(e){const t=e.target.closest("[data-act]");if(!t)return;const a=t.dataset.act;if(a==="toggle")$.enabled=!$.enabled,sn($.enabled),await Oe(),$.enabled?await window.orby.foreground.start()?($.current=await window.orby.foreground.current(),wn($.current)):(g("No se pudo iniciar el detector de aplicaciones","error"),$.enabled=!1,sn(!1),await Oe()):(await window.orby.foreground.stop(),await vt()),Ce();else if(a==="add-current"){const n=$.current?.process;if(!n){g("Aún no se ha detectado ninguna ventana","error");return}if($.rules.some(r=>r.match===n.toLowerCase())){g("Esa aplicación ya tiene tarjeta","info");return}Ua(n.toLowerCase(),s.activeProfileIdx)}else if(a==="add-suggestion")Ua(t.dataset.match,s.activeProfileIdx);else if(a==="add-blank")Ua("",s.activeProfileIdx);else if(a==="remove")$.rules=$.rules.filter(n=>n.id!==t.dataset.id),Oe(),Ce();else if(a==="move-up"||a==="move-down"){const n=$.rules.findIndex(i=>i.id===t.dataset.id),r=a==="move-up"?n-1:n+1;if(n<0||r<0||r>=$.rules.length)return;[$.rules[n],$.rules[r]]=[$.rules[r],$.rules[n]],Oe(),Ce()}}function Ua(e,t){$.rules.push({id:`r${Date.now()}${Math.random().toString(36).slice(2,6)}`,match:e,profile:t??0,field:"any"}),Oe(),Ce()}function Ru(e){const t=e.target,a=t.dataset.id;if(t.dataset.act==="rule-match"||t.dataset.act==="rule-profile"||t.dataset.act==="rule-field"){const n=$.rules.find(r=>r.id===a);if(!n)return;t.dataset.act==="rule-match"&&(n.match=t.value.trim()),t.dataset.act==="rule-profile"&&(n.profile=Number(t.value)),t.dataset.act==="rule-field"&&(n.field=t.value),Oe(),Ce()}else t.dataset.act==="fallback"&&($.fallback=t.value===""?null:Number(t.value),Oe(),Ce())}function Dt(){return s.profiles.length?s.profiles.map(e=>e.name):["P1","P2","P3","P4"]}function Zo(e,t=!1){const a=Dt();return(t?`<option value="" ${e===null?"selected":""}>— no cambiar —</option>`:"")+a.map((r,i)=>`<option value="${i}" ${e===i?"selected":""}>${Z(r)}</option>`).join("")}function ct(){const e=document.getElementById("auto-status");if(!e)return;const t=$.current,a=t?Qo(t):null,n=Dt(),r=hl();e.innerHTML=`
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
          (${Rt(r)}
           ${Rt(r)===1?"cambio":"cambios"})
        </span>`:""}
    </div>
    ${$.error?`<p class="auto-error">${Z($.error)}</p>`:""}`}function Nu(e,t){const a=sa(e.profile),n=Dt(),r=e.match||"Sin programa";return`
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
          ${Zo(e.profile)}
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
    </div>`}function Ce(){const e=document.getElementById("view-auto");if(!e)return;if(!$.available){e.innerHTML=`
      <div class="empty-panel glass-panel">
        ${f("info",40)}
        <h3>Solo disponible en Windows</h3>
        <p>La detección de la ventana activa usa la API de Windows.</p>
      </div>`;return}const t=$.fallback===null||$.fallback===void 0?null:Dt()[$.fallback]??"?";e.innerHTML=`
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
              ${Iu.map(a=>`
                <button class="chip" data-act="add-suggestion" data-match="${a.match}">${a.label}</button>`).join("")}
            </div>
          </div>

          <div class="rule-cards">
            ${$.rules.length?$.rules.map(Nu).join(""):`<div class="rule-empty">
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
              <select class="select-input" data-act="fallback">${Zo($.fallback??null,!0)}</select>
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
          ${Dt().map((a,n)=>`
            <div class="legend-row ${s.activeProfileIdx===n?"on":""}">
              <span class="legend-dot" style="background:${sa(n).accent}"></span>
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
    </div>`,ct()}function Z(e){return String(e??"").replace(/[&<>"]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[t])}const Ou=Object.freeze(Object.defineProperty({__proto__:null,init:Yo,render:Ce},Symbol.toStringTag,{value:"Module"}));let Ga=0,Zr=null;function Jr(e){const t=document.getElementById("backup-status"),a=document.getElementById("btn-backup-save"),n=document.getElementById("btn-backup-load");t&&(t.textContent=e||""),a&&(a.disabled=!!e),n&&(n.disabled=!!e)}function Jo(){const e=(n,r)=>H(n)?!0:(document.querySelector(r)?.classList.add("hidden"),!1),t=e("autostart","#settings-autostart"),a=e("plugins","#settings-plugins");e("appUpdate","#settings-app"),e("firmwareUpdate","#settings-firmware"),document.getElementById("btn-backup-save").addEventListener("click",()=>Ll(Jr)),document.getElementById("btn-backup-load").addEventListener("click",()=>Al(Jr)),document.querySelectorAll("#timeout-selector .opt-btn").forEach(n=>{n.addEventListener("click",async()=>{const r=Number(n.dataset.val);if(Y())try{await cs(r),s.timeout=r,z(),$a()}catch{g("El teclado no confirmó el tiempo de reposo","error")}})}),t&&zu(),a&&Bu(),document.getElementById("btn-reconnect").addEventListener("click",()=>{window.orby.reconnect(),g("Buscando el dispositivo…","info")}),document.getElementById("btn-resync").addEventListener("click",async()=>{try{await fe(),g("Configuración releída del teclado")}catch{g("No se pudo leer la configuración","error")}}),document.getElementById("btn-factory-reset").addEventListener("click",async()=>{if(Y()&&confirm(`Se restaurarán los perfiles, iconos y la calibración de fábrica en la memoria del teclado.

El cambio no será permanente hasta que pulses "Guardar en Flash".

¿Continuar?`))try{await us(),Ma(),await fe(),z(),g("Valores de fábrica restaurados (sin guardar todavía)","info")}catch{g("No se pudo restaurar la configuración","error")}}),Fu(),qu(),_u()}function qu(){const e=document.getElementById("btn-check-update"),t=document.getElementById("btn-install-update");!e||!t||(document.getElementById("app-update-desc").textContent="Se comprueba al arrancar y cada seis horas. Las versiones nuevas se descargan e instalan solas: la app se cierra un momento y vuelve a abrirse sola. Si hay cambios sin guardar en la Flash del teclado, espera a que se guarden.",e.addEventListener("click",async()=>{if(re.status==="dev"){g("En modo desarrollo no hay actualizaciones que buscar","info");return}await Vl(),g("Buscando actualizaciones…","info")}),t.addEventListener("click",()=>Hi()),Di(Mn),Mn())}function Mn(){if(!H("appUpdate"))return;const e=document.getElementById("app-version"),t=document.getElementById("app-update-status"),a=document.getElementById("btn-install-update");if(!e||!t||!a)return;e.textContent=re.version||"—",t.textContent=Fl(),t.style.color=re.status==="error"?"var(--danger)":"";const n=re.status;a.classList.toggle("hidden",n!=="downloaded"&&n!=="available")}function _u(){const e=document.getElementById("settings-firmware");if(e){if(!Ia()){e.classList.add("hidden");return}document.getElementById("btn-fw-check").addEventListener("click",async()=>{const t=await Xi();if(!t){g("La actualización de firmware no está disponible en esta sesión","error",6e3);return}if(t.status==="error"){g(t.error,"error",6e3);return}t.status==="idle"&&!t.available&&g(t.latest?`El teclado ya tiene el último firmware (${t.latest.version})`:"No hay firmware publicado que esta versión de la app sepa instalar","info")}),document.getElementById("btn-fw-update").addEventListener("click",async()=>{if(!Y())return;if(s.dirty){g("Guarda los cambios en Flash antes de actualizar el firmware","error",6e3);return}const a=!st(s.deviceInfo,"bootsel")?`Este firmware no sabe reiniciarse solo: cuando te lo pida, desenchufa el teclado y vuelve a enchufarlo con BOOTSEL pulsado.

`:"";confirm(`${a}El teclado dejará de funcionar durante la copia (unos segundos). No lo desconectes.

¿Actualizar a la ${G.latest?.version}?`)&&(await Jl(),G.status==="done"?g("Firmware actualizado"):G.status==="error"&&g(G.error,"error",9e3))}),document.getElementById("btn-fw-cancel").addEventListener("click",()=>{ec(),g("Actualización cancelada","info")}),Ql(kn),Zl().then(kn)}}function kn(){if(!H("firmwareUpdate"))return;const e=document.getElementById("fw-current");if(!e)return;const t=G,a=document.getElementById("fw-latest"),n=document.getElementById("fw-status"),r=document.getElementById("btn-fw-update"),i=document.getElementById("btn-fw-check"),o=document.getElementById("btn-fw-cancel");e.textContent=s.connected?s.deviceInfo?.fw||"?":"sin teclado",a.textContent=t.latest?.version||"—",n.textContent=tc(),n.style.color=t.status==="error"?"var(--danger)":"",i.disabled=Sr(),o.classList.toggle("hidden",!["downloading","bootsel"].includes(t.status)),r.classList.toggle("hidden",!t.latest||!s.connected||Sr()),document.getElementById("fw-update-label").textContent=t.available?`Actualizar a ${t.latest.version}`:"Reinstalar firmware"}async function zu(){const e=document.getElementById("btn-autostart");e&&(e.classList.toggle("on",await window.orby.autostart.get()),e.addEventListener("click",async()=>{const t=await window.orby.autostart.set(!e.classList.contains("on"));e.classList.toggle("on",t),g(t?"OrbyGUI arrancará con Windows":"Autoarranque desactivado")}))}function Bu(){const e=document.getElementById("btn-plugin-install"),t=document.getElementById("btn-plugin-folder"),a=document.getElementById("plugin-list"),n=document.getElementById("plugin-settings-cards");!e||!a||!n||(e.addEventListener("click",async()=>{e.disabled=!0;const r=await window.orby.plugins.install();if(e.disabled=!1,!r.canceled){if(!r.ok){g(`No se pudo instalar: ${r.error}`,"error",6e3);return}await ra(),g(`Complemento «${r.plugin.name}» instalado`)}}),t?.addEventListener("click",()=>window.orby.plugins.openFolder()),a.addEventListener("click",async r=>{const i=r.target.closest("[data-act]");if(!i)return;const o=i.dataset.plugin;if(i.dataset.act==="plugin-enable"){const c=Ze(o);await window.orby.plugins.setEnabled(o,!c?.enabled),await ra()}else if(i.dataset.act==="plugin-remove"){const c=Ze(o);if(!confirm(`Se desinstalará «${c?.name||o}».

Las teclas y mandos que lo usaran dejarán de hacer nada, pero conservarán su ajuste: si vuelves a instalarlo, volverán a funcionar.

¿Continuar?`))return;const u=await window.orby.plugins.uninstall(o);if(!u.ok){g(`No se pudo desinstalar: ${u.error}`,"error",5e3);return}await ra(),g("Complemento desinstalado")}}),n.addEventListener("change",r=>{const i=r.target.closest("[data-plugin][data-key]");if(!i)return;const o=i.type==="number"?Number(i.value):i.value;window.orby.plugins.setSettings(i.dataset.plugin,{[i.dataset.key]:o})}),n.addEventListener("click",async r=>{const i=r.target.closest("[data-act]");if(i)if(i.dataset.act==="plugin-toggle-field"){const o=!i.classList.contains("on");i.classList.toggle("on",o),window.orby.plugins.setSettings(i.dataset.plugin,{[i.dataset.key]:o})}else i.dataset.act==="plugin-test"&&await ju(i.dataset.plugin,i)}),Zn(ei),ei())}async function ju(e,t){const a=document.getElementById(`plugin-status-${e}`),n={};document.querySelectorAll(`#plugin-settings-cards [data-plugin="${e}"][data-key]`).forEach(i=>{n[i.dataset.key]=i.classList?.contains("switch")?i.classList.contains("on"):i.type==="number"?Number(i.value):i.value}),window.orby.plugins.setSettings(e,n),a&&(a.textContent="Probando…"),t.disabled=!0;const r=await window.orby.plugins.test(e,n);t.disabled=!1,a&&(a.textContent=r.ok?r.detail:`Sin respuesta (${r.error})`),g(r.ok?"El complemento responde":"El complemento no responde",r.ok?"success":"error")}function ei(){H("plugins")&&(Du(),Hu())}function Du(){const e=document.getElementById("plugin-list");if(!e)return;const t=Ul();if(!t.length){e.innerHTML='<p class="plugin-empty">Todavía no hay ninguno instalado.</p>';return}e.innerHTML=t.map(a=>`
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
    </div>`).join("")}function Hu(){const e=document.getElementById("plugin-settings-cards");if(!e)return;const t=Jn().filter(a=>a.settings?.fields?.length||a.settings?.hasTest);e.innerHTML=t.map(a=>`
    <div class="settings-card glass-panel">
      <div class="card-header">
        <span>${f(a.icon,22)}</span>
        <h2>${ae(a.name)}</h2>
      </div>
      <div class="setting-body">
        ${(a.settings.fields||[]).map(n=>Vu(a,n)).join("")}
        ${a.settings.hasTest?`
          <div class="row-inline">
            <button class="secondary-btn" data-act="plugin-test" data-plugin="${a.id}">Probar</button>
            <span class="setting-desc" id="plugin-status-${a.id}"></span>
          </div>`:""}
        ${a.settings.description?`<p class="setting-desc">${ae(a.settings.description)}</p>`:""}
      </div>
    </div>`).join("")}function Vu(e,t){const a=e.values?.[t.key],n=`data-plugin="${e.id}" data-key="${ae(t.key)}"`;if(t.type==="toggle")return`
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
    ${t.hint?`<p class="setting-desc">${ae(t.hint)}</p>`:""}`}function ae(e){return String(e??"").replace(/[&<>"']/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[t])}function Fu(){const e=document.getElementById("settings-wheel-calib-body");e&&(e.addEventListener("click",t=>{const a=t.target.closest("[data-act]");if(!a)return;const n=a.dataset.act;n==="dial-marker"?Ka({marker:a.dataset.marker}):n==="dial-invert"?Ka({invert:!$e.invert}):n==="dial-nudge"?Ka({offsetDeg:Gn($e.offsetDeg+Number(a.dataset.d))}):n==="dial-align"&&(dl(),va(),Ta())}),e.addEventListener("input",t=>{if(t.target.dataset.act!=="dial-offset")return;const a=Number(t.target.value);Wn({offsetDeg:a});const n=document.getElementById("dial-offset-val");n&&(n.textContent=`${a}°`)}),Ci(es),W("telemetry",t=>{t.startsWith("WHEEL:")&&Wu(parseInt(t.slice(6),10))}),va())}function es(e){const t=document.getElementById("settings-wheel-needle");t&&(t.style.transform=`rotate(${e}deg)`)}function Wu(e){if(!Number.isFinite(e))return;Ga+=e;const t=document.getElementById("wheel-calib-readout");if(!t)return;const a=s.scroll.detentsPerRev||60,n=Ga*a/4096;t.textContent=`Rueda en ${Gn(Un()).toFixed(0)}°  ·  ${n>=0?"+":""}${n.toFixed(2)} clics en este giro`,clearTimeout(Zr),Zr=setTimeout(()=>{Ga=0},1200)}function Ka(e){Wn(e),va(),Ta()}function va(){const e=document.getElementById("settings-wheel-calib-body");if(!e)return;const t=$e;e.innerHTML=`
    <p class="setting-desc">
      Cómo se dibuja la rueda magnética en pantalla. No afecta al teclado: solo hace que el
      dibujo coincida con el marcador que lleve pegado la tapa.
    </p>

    <div class="wheel-dial">
      <div class="wheel-dial-face">${Kn("settings-wheel-needle")}</div>
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
    </div>`,es(Un())}function $a(){Mn(),kn(),document.querySelectorAll("#timeout-selector .opt-btn").forEach(t=>{t.classList.toggle("active",Number(t.dataset.val)===s.timeout)});const e=document.getElementById("device-info-list");if(e){if(s.connected){const t=s.deviceInfo||{},a=Ki(t),n=a.level==="ok"?"var(--ok, inherit)":"var(--danger)";e.innerHTML=`
      <li><span class="lbl">Dispositivo</span><span class="val">${t.device||"ORBY_V4"}</span></li>
      <li><span class="lbl">Firmware</span><span class="val">${t.fw||"?"}</span></li>
      <li><span class="lbl">Compatibilidad</span><span class="val" style="color:${n}"
          title="${a.detail}">${a.level==="ok"?"al día":a.title}</span></li>
      <li><span class="lbl">Puerto</span><span class="val">${t.port||"—"}</span></li>
      <li><span class="lbl">Teclas / OLEDs</span><span class="val">${t.keys||12} / ${t.oleds||10}</span></li>
      <li><span class="lbl">Perfiles</span><span class="val">${s.profiles.length} / ${s.maxProfiles}</span></li>
      <li><span class="lbl">Modo</span><span class="val">${s.deviceMode}</span></li>
      <li><span class="lbl">Scroll alta res.</span><span class="val">${s.scroll.hires?"sí":"no"}</span></li>`}else e.innerHTML='<li><span class="lbl">Estado</span><span class="val" style="color:var(--danger)">Desconectado</span></li>';va()}}const Uu=Object.freeze(Object.defineProperty({__proto__:null,init:Jo,render:$a},Symbol.toStringTag,{value:"Module"})),ti=500,We=[];let Et=!1,Xa=null,En="";function ts(){const e=document.getElementById("console-input");document.getElementById("btn-clear-console").addEventListener("click",()=>{We.length=0,Pn()});const t=document.getElementById("btn-pause-console");t.addEventListener("click",()=>{Et=!Et,t.classList.toggle("is-on",Et),t.title=Et?"Reanudar":"Pausar"}),document.getElementById("console-filter").addEventListener("input",r=>{En=r.target.value.trim().toUpperCase(),Pn()});const a=[];let n=-1;e.addEventListener("keydown",r=>{if(r.key==="Enter"&&e.value.trim()){const i=e.value.trim();a.unshift(i),n=-1,ls(i),e.value=""}else r.key==="ArrowUp"&&a.length?(n=Math.min(n+1,a.length-1),e.value=a[n],r.preventDefault()):r.key==="ArrowDown"&&(n=Math.max(n-1,-1),e.value=n>=0?a[n]:"",r.preventDefault())}),W("rx",r=>He(r,"rx")),W("tx",r=>He(r,"tx")),W("error",r=>He(`ERROR: ${r}`,"error")),W("connected",r=>He(`Conectado: ${r?.raw||"ORBY_V4"}`,"system")),W("disconnected",()=>He("Dispositivo desconectado.","error")),He("Terminal OrbyGUI. Escribe GET_STATE para volcar la configuración.","system")}function He(e,t="system"){if(Et&&t==="rx")return;const a=new Date().toLocaleTimeString("es-ES",{hour12:!1});We.push({time:a,text:e,type:t}),We.length>ti&&We.splice(0,We.length-ti),Pn()}function Pn(){Xa||(Xa=requestAnimationFrame(()=>{Xa=null,Ku()}))}function Gu(e){return e.replace(/[&<>]/g,t=>({"&":"&amp;","<":"&lt;",">":"&gt;"})[t])}function Ku(){const e=document.getElementById("console-output");if(!e)return;const t=e.scrollHeight-e.scrollTop-e.clientHeight<40,a=En?We.filter(n=>n.text.toUpperCase().includes(En)):We;e.innerHTML=a.map(n=>`<div class="console-line ${n.type}"><span class="ts">${n.time}</span>${Gu(n.text)}</div>`).join(""),t&&(e.scrollTop=e.scrollHeight)}const Xu=Object.freeze(Object.defineProperty({__proto__:null,init:ts,push:He},Symbol.toStringTag,{value:"Module"})),Jt={"view-dashboard":Cc,"view-profiles":Wd,"view-oled":Su,"view-auto":Ou,"view-settings":Uu,"view-console":Xu};let Me="view-dashboard",as=Promise.resolve(),ai=!1;const Yu=2500;function ns(){ai||(ai=!0,ji())}function Pt(e,t="info",a=900){jl()?ua(e):g(e,t,a)}function Qu(){H("windowChrome")||document.querySelector(".titlebar-controls")?.classList.add("hidden"),H("autoProfile")||document.querySelector('.nav-item[data-target="view-auto"]')?.classList.add("hidden"),H("appUpdate")||document.getElementById("btn-update")?.classList.add("hidden"),document.getElementById("btn-minimize").addEventListener("click",()=>window.orby.minimize()),document.getElementById("btn-maximize").addEventListener("click",()=>window.orby.maximize()),document.getElementById("btn-close").addEventListener("click",()=>window.orby.close()),document.querySelectorAll(".nav-item").forEach(e=>{e.addEventListener("click",()=>Cn(e.dataset.target))}),document.getElementById("btn-save-flash").addEventListener("click",rs),Zu()}function Zu(){if(!H("appUpdate"))return;document.getElementById("btn-update").addEventListener("click",async()=>{const{status:t,newVersion:a}=re;t==="downloaded"&&confirm(`Se instalará OrbyGUI ${a}.

La app se cerrará y volverá a abrirse sola. ¿Continuar?`)&&await Hi()}),Di(ni),Dl(),ni()}function ni(){if(!H("appUpdate"))return;const e=document.getElementById("btn-update"),t=e.querySelector(".update-label"),{status:a,newVersion:n,percent:r}=re;e.classList.toggle("hidden",a!=="downloading"&&a!=="downloaded"),e.classList.toggle("ready",a==="downloaded"),a==="downloading"?(t.textContent=`Descargando ${r}%`,e.title=`Bajando OrbyGUI ${n}`):a==="downloaded"&&(t.textContent=`Actualizar a ${n}`,e.title=`OrbyGUI ${n} descargada. Se instalará sola en cuanto se guarden los cambios en la Flash; haz clic para instalarla ahora`)}const Ju={"view-oled":"view-profiles"};function Cn(e,t){Me=e;const a=Ju[e]||e;document.querySelectorAll(".nav-item").forEach(n=>n.classList.toggle("active",n.dataset.target===a)),document.querySelectorAll(".view").forEach(n=>n.classList.toggle("active",n.id===e)),t&&e==="view-oled"?zo(t):Jt[e]?.render?.()}async function rs(){const e=document.getElementById("btn-save-flash");if(s.connected){Ue&&(clearTimeout(Ue),Ue=null),e.disabled=!0;try{await kl(async()=>{await ds(),s.dirty=!1,te()})}catch{g("El teclado no confirmó el guardado, reintentando…","error"),is()}finally{e.disabled=!1}}}const ep=1500;let Ue=null;function is(){Ue&&clearTimeout(Ue),Ue=setTimeout(()=>{Ue=null,rs()},ep)}function ri(){const e=document.getElementById("connection-status-badge"),t=e.querySelector(".status-text");if(s.connected){e.className="status-badge connected";const n=s.deviceInfo?.fw?` · fw ${s.deviceInfo.fw}`:"";t.textContent=s.syncing?"Sincronizando…":`Orby V4 conectado${n}`}else e.className="status-badge disconnected",t.textContent="Desconectado";document.getElementById("readonly-banner").classList.toggle("hidden",s.connected),document.body.classList.toggle("read-only",!s.connected),s.dirty&&s.connected&&is();const a=document.getElementById("btn-save-flash");a.classList.toggle("has-changes",s.dirty),a.querySelector(".save-label").textContent=s.dirty?"Guardando…":"Guardado",a.disabled=!s.connected}const tp=20;function ap(e){if(!e){je(s.activeProfileIdx);return}al((t,a)=>{t%tp===0&&na(),t===a&&Pt("Iconos descargados","info",1500)}).then(()=>{Jt[Me]?.render?.(),na()}).catch(t=>{console.error("No se pudieron precargar los iconos:",t),na()})}async function np(e=3){for(let t=0;t<e;t++){const a=await vs();if(a?.all)return a.per;if(!Sn())return null;await new Promise(n=>setTimeout(n,400))}return null}async function rp(e){s.connected=!0,s.deviceInfo=e,te();const t=Ki(e);if(t.level==="blocked"){g(`${t.title}. ${t.detail}`,"error",9e3);return}t.level!=="ok"&&g(`${t.title}. ${t.detail}`,"error",9e3),await as;const a=st(e,"hash");try{let n=null;a&&(Pt("Comprobando si la copia del PC sigue valiendo…"),n=await np(),n||(console.warn("[sync] el teclado no ha dado las huellas: toca releerlo todo"),Pt("El teclado no ha dado las huellas: hay que releerlo todo","error",5e3)));const r=await fe({expected:n,iconOf:ye,onProgress:(i,o)=>Pt(`Leyendo perfil ${i+1} de ${o}…`)});for(const i of r)Ei(i);Js(s.profiles.length),Jt[Me]?.render?.(),r.length?Pt("Perfiles cargados desde el teclado","success",2600):ua("Todo al día")}catch(n){g(`No se pudo leer la configuración: ${n.message}`,"error");return}ap(a),no?.().catch(()=>{}),H("firmwareUpdate")&&Xi()?.then(n=>{n?.available&&g(`Hay firmware nuevo para el teclado (${n.latest.version}). Ajustes → Firmware del teclado`,"info",8e3)}).catch(()=>{})}function ip(){W("connected",e=>{rp(e).catch(t=>{console.error("Fallo al preparar la conexión con el teclado:",t)}).finally(ns)}),W("disconnected",()=>{s.connected=!1,s.deviceInfo=null,te()}),W("telemetry",e=>{if(!e.startsWith("EV:CTX:"))return;const[t,a,n]=e.slice(7).split(":").map(Number);Ts(t,a,n)&&(je(t),Jt[Me]?.render?.()),Ml().catch(()=>{})}),W("searching",()=>{if(s.connected)return;const e=document.getElementById("connection-status-badge");e.className="status-badge searching",e.querySelector(".status-text").textContent="Buscando USB…"})}function ii(){Bl(),nl(),Qu(),Ms(),ip(),il(Cn),ol().then(()=>{Ta(),Me==="view-profiles"&&C(),Me==="view-settings"&&$a()}),Wl(),Mc(),pl().then(()=>{Me==="view-profiles"&&C()}),ql(),ua("Leyendo la copia del PC…"),as=Rl().then(e=>{e&&Jt[Me]?.render?.(),ua(s.connected?"Hablando con el teclado…":"Buscando el teclado…"),setTimeout(()=>{s.connected||(ns(),e&&g("Configuración cargada de la copia del PC; conecta el Orby para editarla","info",5e3))},Yu)}),oo(),Eo(),_o(),Jo(),ts(),H("autoProfile")&&Yo(),rt(()=>{ri(),$a(),Me==="view-dashboard"&&Ge()}),ri(),Cn("view-dashboard")}document.readyState==="loading"?document.addEventListener("DOMContentLoaded",ii):ii();
