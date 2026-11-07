// Calls the dashboard's own JavaScript functions under node, for tests that need no browser. Run by
// tests/test_dashboard_resize.py:
//   node dashboard_calls.mjs <dashboard dir> '[["resize.js", "expression", ["sensors", 700]], ["resize.js", "SIZES"], ...]'
// Prints, in order, each function's result for its arguments, or the export itself when it is not a function.
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';

const [directory, calls] = [process.argv[2], JSON.parse(process.argv[3])];
const results = [];
for (const [file, name, args = []] of calls) {
  const module = await import(pathToFileURL(join(directory, file)).href);
  results.push(typeof module[name] === 'function' ? module[name](...args) : module[name]);
}
console.log(JSON.stringify(results));
