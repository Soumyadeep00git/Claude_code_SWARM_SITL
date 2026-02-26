// Network Mesh Visualization — Leaflet overlay + stats panel
// Loaded after swarm.js, shares: map, socket, COLORS, addLog, latestState

(function () {
    "use strict";

    // ── State ──────────────────────────────────────────
    let networkVisible = true;
    let linkLines = [];       // Leaflet polylines
    let latestNetwork = null; // last network_update payload

    // ── Link color from quality ────────────────────────
    function qualityColor(q) {
        if (q >= 0.7) return "#22c55e";  // green
        if (q >= 0.4) return "#f59e0b";  // amber
        if (q > 0)    return "#ef4444";  // red
        return "#64748b";                 // gray (no link)
    }

    function qualityWidth(q) {
        return Math.max(1, Math.round(q * 4));
    }

    // ── Draw network overlay on map ────────────────────
    function drawNetwork(data) {
        clearLinks();
        if (!networkVisible || !data || !data.links) return;

        const drones = (latestState && latestState.drones) || {};

        for (const link of data.links) {
            const srcState = drones[String(link.src)];
            const dstState = drones[String(link.dst)];
            if (!srcState || !dstState) continue;
            if (srcState.lat === 0 && srcState.lon === 0) continue;
            if (dstState.lat === 0 && dstState.lon === 0) continue;

            const q = link.quality || 0;
            const line = L.polyline(
                [[srcState.lat, srcState.lon], [dstState.lat, dstState.lon]],
                {
                    color: qualityColor(q),
                    weight: qualityWidth(q),
                    opacity: 0.7,
                    dashArray: q < 0.2 ? "4,6" : null,
                }
            ).addTo(map);

            const pct = (q * 100).toFixed(0);
            line.bindTooltip(
                `D${link.src}\u2194D${link.dst} | ${pct}% | ${link.distance_m}m`,
                { sticky: true, className: "drone-tooltip" }
            );
            linkLines.push(line);
        }
    }

    function clearLinks() {
        for (const l of linkLines) map.removeLayer(l);
        linkLines = [];
    }

    // ── Stats panel rendering ──────────────────────────
    function renderNodeStats(data) {
        const container = document.getElementById("net-node-stats");
        if (!container || !data || !data.nodes) {
            if (container) container.innerHTML = '<span class="hint">No mesh data yet</span>';
            return;
        }

        const ids = Object.keys(data.nodes).map(Number).sort((a, b) => a - b);
        if (ids.length === 0) {
            container.innerHTML = '<span class="hint">No mesh data yet</span>';
            return;
        }

        let html = '<table class="net-stats-table"><thead><tr>' +
            '<th>ID</th><th>TX</th><th>RX</th><th>DROP</th><th>FWD</th><th>Loss</th>' +
            '</tr></thead><tbody>';

        for (const did of ids) {
            const s = data.nodes[did];
            const color = COLORS[(did - 1) % COLORS.length];
            const loss = s.sent > 0 ? ((s.dropped / s.sent) * 100).toFixed(1) : "0.0";
            const lossClass = parseFloat(loss) > 10 ? "net-loss-high" :
                              parseFloat(loss) > 2 ? "net-loss-med" : "net-loss-ok";
            const rowClass = s.stale ? ' class="stale-row"' : '';
            html += `<tr${rowClass}>
                <td style="color:${color};font-weight:600">${s.stale ? "\u26A0 " : ""}D${did}</td>
                <td>${s.sent}</td><td>${s.delivered}</td>
                <td>${s.dropped}</td><td>${s.forwarded}</td>
                <td class="${lossClass}">${s.stale ? "LOST" : loss + "%"}</td>
            </tr>`;
        }
        html += '</tbody></table>';
        container.innerHTML = html;
    }

    function renderRoutingTable(data) {
        const container = document.getElementById("net-routing");
        if (!container) return;

        const sel = document.getElementById("net-route-drone");
        if (!sel || !data || !data.routing_tables) {
            if (container) container.innerHTML = '<span class="hint">No routes</span>';
            return;
        }

        const did = parseInt(sel.value);
        const rt = data.routing_tables[did];
        if (!rt || Object.keys(rt).length === 0) {
            container.innerHTML = '<span class="hint">No routes for D' + did + '</span>';
            return;
        }

        let html = '<table class="net-stats-table"><thead><tr>' +
            '<th>Dest</th><th>Next</th><th>Hops</th><th>Cost</th>' +
            '</tr></thead><tbody>';
        for (const [dest, info] of Object.entries(rt).sort((a, b) => parseInt(a[0]) - parseInt(b[0]))) {
            const dc = COLORS[(parseInt(dest) - 1) % COLORS.length];
            const nc = COLORS[(info.next_hop - 1) % COLORS.length];
            html += `<tr>
                <td style="color:${dc}">D${dest}</td>
                <td style="color:${nc}">D${info.next_hop}</td>
                <td>${info.hops}</td>
                <td>${info.metric}</td>
            </tr>`;
        }
        html += '</tbody></table>';
        container.innerHTML = html;
    }

    // ── Route trace on map ─────────────────────────────
    let traceLines = [];
    function traceRoute(data) {
        // Clear previous trace
        for (const l of traceLines) map.removeLayer(l);
        traceLines = [];

        if (!data || !data.routing_tables) return;

        const srcId = parseInt(document.getElementById("net-trace-src").value);
        const dstId = parseInt(document.getElementById("net-trace-dst").value);
        if (isNaN(srcId) || isNaN(dstId) || srcId === dstId) return;

        const drones = (latestState && latestState.drones) || {};

        // Walk the routing table from src to dst
        const path = [srcId];
        let current = srcId;
        const visited = new Set();
        for (let i = 0; i < 10; i++) {
            visited.add(current);
            const rt = data.routing_tables[current];
            if (!rt || !rt[String(dstId)]) break;
            const nextHop = rt[String(dstId)].next_hop;
            path.push(nextHop);
            if (nextHop === dstId) break;
            if (visited.has(nextHop)) break; // loop
            current = nextHop;
        }

        if (path.length < 2) {
            addLog("WARN", `No route from D${srcId} to D${dstId}`);
            return;
        }

        // Draw animated path
        for (let i = 0; i < path.length - 1; i++) {
            const s = drones[String(path[i])];
            const d = drones[String(path[i + 1])];
            if (!s || !d || (s.lat === 0 && s.lon === 0)) continue;

            const line = L.polyline(
                [[s.lat, s.lon], [d.lat, d.lon]],
                {
                    color: "#a78bfa",
                    weight: 4,
                    opacity: 0.9,
                    dashArray: "8,6",
                    className: "trace-line-animated",
                }
            ).addTo(map);
            line.bindTooltip(`Hop ${i + 1}: D${path[i]}\u2192D${path[i + 1]}`,
                { sticky: true, className: "drone-tooltip" });
            traceLines.push(line);
        }

        addLog("INFO", `Route trace: ${path.map(p => "D" + p).join(" \u2192 ")}`);
    }

    // ── Populate selects ───────────────────────────────
    function updateNetSelects() {
        const ids = [];
        for (let i = 1; i <= MAX_DRONES; i++) ids.push(i);

        ["net-route-drone", "net-trace-src", "net-trace-dst"].forEach(selId => {
            const sel = document.getElementById(selId);
            if (!sel || sel.options.length === MAX_DRONES) return;
            sel.innerHTML = "";
            for (const i of ids) {
                const o = document.createElement("option");
                o.value = i;
                o.textContent = `D${i}`;
                sel.appendChild(o);
            }
        });

        // Default trace dst to last drone
        const dst = document.getElementById("net-trace-dst");
        if (dst && dst.options.length > 0) dst.value = MAX_DRONES;
    }

    // ── SocketIO ───────────────────────────────────────
    socket.on("network_update", (data) => {
        latestNetwork = data;
        drawNetwork(data);
        renderNodeStats(data);
        renderRoutingTable(data);
    });

    // ── Button handlers ────────────────────────────────
    document.addEventListener("DOMContentLoaded", () => { setTimeout(initNetwork, 100); });
    if (document.readyState !== "loading") setTimeout(initNetwork, 100);

    function initNetwork() {
        updateNetSelects();

        // Network overlay toggle
        const btnToggle = document.getElementById("btn-toggle-network");
        if (btnToggle) {
            btnToggle.onclick = () => {
                networkVisible = !networkVisible;
                btnToggle.classList.toggle("active", networkVisible);
                if (!networkVisible) {
                    clearLinks();
                    for (const l of traceLines) map.removeLayer(l);
                    traceLines = [];
                } else if (latestNetwork) {
                    drawNetwork(latestNetwork);
                }
            };
        }

        // Range slider
        const rangeSlider = document.getElementById("net-range-slider");
        const rangeVal = document.getElementById("net-range-val");
        if (rangeSlider) {
            rangeSlider.oninput = () => {
                if (rangeVal) rangeVal.textContent = rangeSlider.value;
            };
            rangeSlider.onchange = () => {
                const range = parseInt(rangeSlider.value);
                socket.emit("cmd_set_mesh_range", { range_m: range });
                addLog("INFO", `Mesh range set to ${range}m`);
            };
        }

        // Routing table drone select
        const rtSel = document.getElementById("net-route-drone");
        if (rtSel) {
            rtSel.onchange = () => { if (latestNetwork) renderRoutingTable(latestNetwork); };
        }

        // Trace route button
        const btnTrace = document.getElementById("btn-trace-route");
        if (btnTrace) {
            btnTrace.onclick = () => { if (latestNetwork) traceRoute(latestNetwork); };
        }
    }
})();
