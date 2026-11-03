(function(){"use strict";let s=null;self.onmessage=e=>{clearInterval(s),s=null,e.data?.ms&&(s=setInterval(()=>self.postMessage(0),e.data.ms))}})();
