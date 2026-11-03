// Page writes that happen only when a value changes. robot_state arrives 20 times a second and most of what
// the dashboard shows is the same from one message to the next, so unchanged text and properties are left alone.

// An element's text, edited in its one text node rather than replaced.
export function setText(element, text) {
  const node = element.firstChild;
  if (node?.nodeType === Node.TEXT_NODE && !node.nextSibling) { if (node.data !== text) node.data = text; }
  else if (element.textContent !== text) element.textContent = text;
}

// A property such as className, title or hidden.
export function setProperty(element, name, value) {
  if (element[name] !== value) element[name] = value;
}

// The children of list as one group of elements per key, in the order of keys. Each group is made once by
// make(key), stays while its key does, and is moved only when the order changes. made: the Map of groups.
export function placeKeyed(list, made, keys, make) {
  for (const [key, elements] of made) {
    if (keys.includes(key)) continue;
    for (const element of elements) element.remove();
    made.delete(key);
  }
  let next = list.firstChild;
  for (const key of keys) {
    if (!made.has(key)) made.set(key, make(key));
    for (const element of made.get(key)) {
      if (element === next) next = next.nextSibling;
      else list.insertBefore(element, next);
    }
  }
}
