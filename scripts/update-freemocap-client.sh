#!/usr/bin/env bash
# Rebuild @freemocap/client's browser bundle from a FreeMoCap checkout and vendor it
# for the dashboard, which loads it without a bundler (the Pi has no Node).
#
#   scripts/update-freemocap-client.sh /path/to/freemocap-checkout
#
# Use the checkout matching the freemocap-stream commit in uv.lock, so the browser
# and the server speak the same version of the stream.
set -euo pipefail

freemocap="${1:?usage: $0 /path/to/freemocap-checkout}"
client="$freemocap/packages/freemocap-client"
target="$(cd "$(dirname "$0")/.." && pwd)/ultraskelly/ui/dashboard/vendor"

(cd "$client" && npm ci --ignore-scripts --no-audit --no-fund && npm run build:browser)
mkdir -p "$target"
cp "$client/dist/browser/freemocap-client.js" "$client/dist/browser/freemocap-client.js.map" "$target/"
cat > "$target/freemocap-client.SOURCE" <<EOF
@freemocap/client browser bundle (AGPL-3.0-or-later; bundles zod and cbor-x, both MIT)
built by scripts/update-freemocap-client.sh from packages/freemocap-client
freemocap $(git -C "$freemocap" rev-parse --abbrev-ref HEAD) $(git -C "$freemocap" rev-parse HEAD)
EOF
echo "Vendored the bundle into $target"
