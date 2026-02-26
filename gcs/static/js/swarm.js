// Swarm SITL — Web GCS (Leaflet + floating overlays)

// ── Constants ──────────────────────────────────────────
const COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00",
                "#a65628", "#f781bf", "#999999", "#66c2a5", "#fc8d62"];
const HOME_LAT = -35.3632620;
const HOME_LON = 149.1652370;
const TRAIL_LENGTH = 50;
const AUTO_CENTER_INTERVAL = 3000;

// ── State ──────────────────────────────────────────────
let latestState = null;
let processStatus = {};
let knownDrones = [];
let autoCenter = true;
let showTrails = true;
let mapInitialized = false;
let lastCenterTime = 0;

let map;
let droneMarkers = {};
let droneIconCache = {};
let droneTrails = {};
let waypointMarker = null;

// Isolation zone visualization
let showIsolationZones = false;
let isolationCircles = {};
let currentIsolationRadius = 5.0;

// Proximity alert flash state
let proxAlertFlash = {};  // drone_id -> expiry timestamp

// ── DOM ────────────────────────────────────────────────
const statusBody = document.getElementById("status-body");
const logScroll = document.getElementById("log-scroll");
const connStatus = document.getElementById("conn-status");
const elapsedSpan = document.getElementById("elapsed");

// ── Map ────────────────────────────────────────────────

function initMap() {
    const tileDark = L.tileLayer(
        "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",
        { attribution: "&copy; CartoDB", subdomains: "abcd", maxZoom: 20 }
    );
    const tileSat = L.tileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        { attribution: "&copy; Esri", maxZoom: 19 }
    );

    map = L.map("map", {
        center: [HOME_LAT, HOME_LON],
        zoom: 18,
        layers: [tileDark],
        zoomControl: false,
    });

    L.control.zoom({ position: "topright" }).addTo(map);
    L.control.layers({ "Dark": tileDark, "Satellite": tileSat }, null, { position: "topright" }).addTo(map);

    map.on("dragstart", () => {
        autoCenter = false;
        document.getElementById("btn-auto-center").classList.remove("active");
    });
    map.on("click", onMapClick);
}

// ── Drone Icon ─────────────────────────────────────────

function createDroneIcon(id, heading, isLeader, state, isStale, isBlocked, color, alt, fs) {
    const S = 48, H = S / 2;
    const rad = (heading || 0) * Math.PI / 180;
    const lx = H + Math.sin(rad) * 20;
    const ly = H - Math.cos(rad) * 20;

    const isProxAlert = proxAlertFlash[id] && proxAlertFlash[id] > Date.now();

    let extra = "";
    if (isProxAlert) extra += `<circle cx="${H}" cy="${H}" r="20" fill="none" stroke="#ef4444" stroke-width="3" opacity=".9" class="prox-pulse"/>`;
    if (state && state !== "NOMINAL") {
        const c = (state === "COMMS_LOST" || state.startsWith("EMERGENCY")) ? "#ef4444" : "#f59e0b";
        extra += `<circle cx="${H}" cy="${H}" r="16" fill="none" stroke="${c}" stroke-width="2.5" opacity=".8"/>`;
    }
    if (isBlocked) extra += `<circle cx="${H}" cy="${H}" r="18" fill="none" stroke="#ff6b6b" stroke-width="2" stroke-dasharray="4,4" opacity=".7"/>`;
    if (isStale) extra += `<line x1="${H-8}" y1="${H-8}" x2="${H+8}" y2="${H+8}" stroke="#ef4444" stroke-width="3"/><line x1="${H+8}" y1="${H-8}" x2="${H-8}" y2="${H+8}" stroke="#ef4444" stroke-width="3"/>`;
    if (isLeader) extra += `<text x="${H}" y="${H-18}" text-anchor="middle" fill="#fbbf24" font-size="14" font-weight="bold">\u2605</text>`;
    if (fs && (!state || state === "NOMINAL")) extra += `<text x="${H}" y="${H+30}" text-anchor="middle" fill="#ef4444" font-size="8" font-weight="bold">FS!</text>`;
    // RL mode badge
    const droneRLState = latestState && latestState.drones && latestState.drones[String(id)];
    if (droneRLState && droneRLState.rl_mode) {
        extra += `<rect x="${H-10}" y="${H+14}" width="20" height="10" rx="2" fill="rgba(139,92,246,0.85)"/>`;
        extra += `<text x="${H}" y="${H+22}" text-anchor="middle" fill="#fff" font-size="7" font-weight="bold">RL</text>`;
    }

    const op = isStale ? 0.3 : 1.0;
    const svg = `<svg width="${S}" height="${S}" xmlns="http://www.w3.org/2000/svg" style="opacity:${op};filter:drop-shadow(0 2px 4px rgba(0,0,0,.6))">
        ${extra}
        <circle cx="${H}" cy="${H}" r="10" fill="${color}" stroke="rgba(255,255,255,.45)" stroke-width="1.5"/>
        <line x1="${H}" y1="${H}" x2="${lx}" y2="${ly}" stroke="${color}" stroke-width="2.5" stroke-linecap="round"/>
    </svg>`;

    const altTxt = alt !== undefined ? ` ${alt.toFixed(1)}m` : "";
    return L.divIcon({
        html: `<div class="drone-marker-wrapper">${svg}<span class="drone-label" style="color:${color}">D${id}${altTxt}</span></div>`,
        className: "drone-marker",
        iconSize: [S, S],
        iconAnchor: [H, H],
    });
}

// ── Trails ─────────────────────────────────────────────

function updateTrail(id, lat, lon, color) {
    if (!droneTrails[id]) {
        droneTrails[id] = {
            pts: [],
            line: L.polyline([], { color, weight: 2, opacity: 0.35, smoothFactor: 1 }).addTo(map),
        };
    }
    const t = droneTrails[id];
    t.pts.push([lat, lon]);
    if (t.pts.length > TRAIL_LENGTH) t.pts.shift();
    t.line.setLatLngs(t.pts);
}

function clearTrails() {
    Object.values(droneTrails).forEach(t => map.removeLayer(t.line));
    droneTrails = {};
}

// ── Map Update ─────────────────────────────────────────

function updateMap(data) {
    const drones = data.drones;
    const staleSet = new Set(data.stale);
    const blockedSet = new Set(data.blocked);

    let leaderId = 1;
    for (const [, s] of Object.entries(drones)) { if (s.leader_id) { leaderId = s.leader_id; break; } }

    const bounds = [];
    let count = 0, altSum = 0;

    for (const [id, s] of Object.entries(drones)) {
        const nid = parseInt(id);
        if (s.lat === 0 && s.lon === 0) continue;
        const ll = [s.lat, s.lon];
        const color = COLORS[(nid - 1) % COLORS.length];
        const stale = staleSet.has(nid), blocked = blockedSet.has(nid);
        const leader = nid === leaderId;
        const st = (s.swarm_state || "NOMINAL").toUpperCase();

        // Icon caching
        const hb = Math.round((s.heading || 0) / 5) * 5;
        const pa = proxAlertFlash[nid] && proxAlertFlash[nid] > Date.now() ? 1 : 0;
        const rl = s.rl_mode ? 1 : 0;
        const key = `${nid}_${hb}_${leader}_${st}_${stale}_${blocked}_${(s.alt||0).toFixed(0)}_${s.failsafe_active}_${pa}_${rl}`;

        if (droneMarkers[nid]) {
            droneMarkers[nid].setLatLng(ll);
            if (droneIconCache[nid] !== key) {
                droneMarkers[nid].setIcon(createDroneIcon(nid, s.heading || 0, leader, st, stale, blocked, color, s.alt, s.failsafe_active));
                droneIconCache[nid] = key;
            }
        } else {
            droneMarkers[nid] = L.marker(ll, {
                icon: createDroneIcon(nid, s.heading || 0, leader, st, stale, blocked, color, s.alt, s.failsafe_active),
                zIndexOffset: 1000,
            }).addTo(map);
            droneIconCache[nid] = key;
            droneMarkers[nid].bindTooltip("", { permanent: false, direction: "top", offset: [0, -24], className: "drone-tooltip" });
        }

        let tip = `D${id} | ${s.mode || "?"} | ${(s.alt || 0).toFixed(1)}m`;
        if (st !== "NOMINAL") tip += ` | ${st}`;
        if (s.battery_pct >= 0) tip += ` | ${s.battery_pct}%`;
        droneMarkers[nid].setTooltipContent(tip);

        if (showTrails && !stale) updateTrail(nid, s.lat, s.lon, color);

        bounds.push(ll);
        count++;
        if (s.alt > 0) altSum += s.alt;
    }

    // Isolation zones
    if (showIsolationZones) updateIsolationZones(data);

    // HUD stats
    document.getElementById("drone-count").textContent = `${count}/${MAX_DRONES}`;
    document.getElementById("avg-alt").textContent = count > 0 ? `${(altSum / count).toFixed(1)}m` : "0.0m";

    // Auto-center (throttled, only when drones leave view)
    if (autoCenter && bounds.length > 0) {
        const now = Date.now();
        if (!mapInitialized) {
            map.setView(bounds[0], 18);
            mapInitialized = true;
            lastCenterTime = now;
        } else if (now - lastCenterTime > AUTO_CENTER_INTERVAL) {
            lastCenterTime = now;
            if (bounds.length > 1) {
                const b = L.latLngBounds(bounds).pad(0.3);
                if (!map.getBounds().contains(b)) map.fitBounds(b, { animate: true, duration: 0.8, maxZoom: 19 });
            } else {
                if (!map.getBounds().pad(-0.3).contains(bounds[0])) map.panTo(bounds[0], { animate: true, duration: 0.8 });
            }
        }
    }
}

// ── Isolation Zones ───────────────────────────────────

function updateIsolationZones(data) {
    if (!showIsolationZones) return;
    const drones = data.drones;
    const now = Date.now();

    for (const [id, s] of Object.entries(drones)) {
        const nid = parseInt(id);
        if (s.lat === 0 && s.lon === 0) continue;

        const isFlashing = proxAlertFlash[nid] && proxAlertFlash[nid] > now;
        const color = isFlashing ? "#ef4444" : "rgba(239, 68, 68, 0.3)";
        const fillColor = isFlashing ? "rgba(239, 68, 68, 0.15)" : "rgba(239, 68, 68, 0.05)";
        const weight = isFlashing ? 2.5 : 1.5;

        if (isolationCircles[nid]) {
            isolationCircles[nid].setLatLng([s.lat, s.lon]);
            isolationCircles[nid].setRadius(currentIsolationRadius);
            isolationCircles[nid].setStyle({ color, fillColor, weight });
        } else {
            isolationCircles[nid] = L.circle([s.lat, s.lon], {
                radius: currentIsolationRadius,
                color,
                fillColor,
                fillOpacity: 1,
                weight,
                dashArray: "6,4",
            }).addTo(map);
        }
    }
}

function clearIsolationZones() {
    Object.values(isolationCircles).forEach(c => map.removeLayer(c));
    isolationCircles = {};
}

// ── Click-to-waypoint ──────────────────────────────────

function onMapClick(e) {
    document.getElementById("wp-lat").value = e.latlng.lat.toFixed(7);
    document.getElementById("wp-lon").value = e.latlng.lng.toFixed(7);
    document.getElementById("swarm-wp-lat").value = e.latlng.lat.toFixed(7);
    document.getElementById("swarm-wp-lon").value = e.latlng.lng.toFixed(7);
    if (waypointMarker) map.removeLayer(waypointMarker);
    waypointMarker = L.circleMarker([e.latlng.lat, e.latlng.lng], {
        radius: 8, color: "#fff", fillColor: "#60a5fa", fillOpacity: 0.8, weight: 2,
    }).addTo(map).bindTooltip("Waypoint", { permanent: true, direction: "top", className: "wp-tooltip" });
    setTimeout(() => { if (waypointMarker) { map.removeLayer(waypointMarker); waypointMarker = null; } }, 10000);
}

// ── Socket.IO ──────────────────────────────────────────

const socket = io();

socket.on("connect", () => {
    connStatus.innerHTML = '<span class="conn-dot"></span> Connected';
    connStatus.className = "conn-badge connected";
    addLog("INFO", "Connected to GCS");
});

socket.on("disconnect", () => {
    connStatus.innerHTML = '<span class="conn-dot"></span> Disconnected';
    connStatus.className = "conn-badge disconnected";
    addLog("WARN", "Disconnected from GCS");
});

socket.on("state_update", (data) => {
    latestState = data;
    processStatus = data.process_status || {};
    knownDrones = data.known_drones || [];
    updateElapsed(data.elapsed_s);
    updateStatusTable(data);
    renderDroneGrid();
    renderRLGrid();
    updateDroneSelects();
    updateMap(data);
});

socket.on("alert", (d) => addLog("ALERT", `D${d.drone_id} [${d.code}] ${d.message} -> ${d.action_taken}`));
socket.on("log_event", (d) => addLog(d.level, d.message));
socket.on("drone_launched", (d) => {
    const pidStr = (d.pid === 0 || d.pid === "docker") ? "docker" : `PID ${d.pid}`;
    addLog("INFO", `Drone ${d.drone_id} launched (${pidStr})`);
});
socket.on("drone_killed", (d) => addLog("WARN", `Drone ${d.drone_id} killed`));

socket.on("proximity_alert", (d) => {
    const sev = d.severity || "WARNING";
    const peers = (d.close_peers || []).map(p => "D" + p).join(", ");
    addLog("ALERT", `PROXIMITY D${d.alerting_drone} [${sev}] near ${peers} (iso=${d.isolation_radius}m)`);
    // Flash the alerting drone and close peers on map
    const now = Date.now();
    const flashDuration = sev === "CRITICAL" ? 4000 : 2000;
    proxAlertFlash[d.alerting_drone] = now + flashDuration;
    for (const pid of (d.close_peers || [])) {
        proxAlertFlash[pid] = now + flashDuration;
    }
});

// ── Drone Management Grid ──────────────────────────────

function renderDroneGrid() {
    const grid = document.getElementById("drone-mgmt-grid");
    const isRunning = (st) => st === "running";
    const isDocker = (st) => st === "docker";

    // Update "Launch All" / "Kill All" buttons based on overall fleet status
    let allRunning = true, anyRunning = false;
    let dockerMode = false;
    for (let i = 1; i <= MAX_DRONES; i++) {
        const st = processStatus[i] || "unmanaged";
        if (isDocker(st)) dockerMode = true;
        if (isRunning(st)) anyRunning = true; else allRunning = false;
    }
    const btnLA = document.getElementById("btn-launch-all");
    const btnKA = document.getElementById("btn-kill-all");
    // In Docker mode, Launch All registers drones (always available)
    if (btnLA) btnLA.disabled = !dockerMode && allRunning;
    if (btnKA) btnKA.disabled = dockerMode || !anyRunning;

    if (grid.dataset.count === String(MAX_DRONES)) {
        for (let i = 1; i <= MAX_DRONES; i++) {
            const dot = document.getElementById(`mgmt-dot-${i}`);
            const st = processStatus[i] || "unmanaged";
            if (dot) { dot.className = `status-dot status-${st}`; dot.title = st; }
            const bL = document.getElementById(`mgmt-launch-${i}`);
            const bK = document.getElementById(`mgmt-kill-${i}`);
            const bT = document.getElementById(`mgmt-takeoff-${i}`);
            const bD = document.getElementById(`mgmt-land-${i}`);
            if (bL) bL.disabled = isRunning(st);
            if (bK) bK.disabled = !isRunning(st) || isDocker(st);
            const has = latestState && latestState.drones && latestState.drones[String(i)];
            if (bT) bT.disabled = !has;
            if (bD) bD.disabled = !has;
        }
        return;
    }
    grid.innerHTML = "";
    grid.dataset.count = String(MAX_DRONES);
    for (let i = 1; i <= MAX_DRONES; i++) {
        const st = processStatus[i] || "unmanaged";
        const c = COLORS[(i - 1) % COLORS.length];
        const row = document.createElement("div");
        row.className = "drone-mgmt-row";
        row.innerHTML = `
            <span class="status-dot status-${st}" id="mgmt-dot-${i}" title="${st}"></span>
            <span style="color:${c};font-weight:600;width:26px;font-family:var(--font-mono);font-size:11px">D${i}</span>
            <button class="btn btn-success btn-sm" id="mgmt-launch-${i}" ${isRunning(st)?"disabled":""}>Launch</button>
            <button class="btn btn-danger btn-sm" id="mgmt-kill-${i}" ${(!isRunning(st)||isDocker(st))?"disabled":""}>Kill</button>
            <button class="btn btn-primary btn-sm" id="mgmt-takeoff-${i}" disabled>Up</button>
            <button class="btn btn-sm" id="mgmt-land-${i}" disabled>Land</button>`;
        grid.appendChild(row);
        document.getElementById(`mgmt-launch-${i}`).onclick = () => socket.emit("cmd_launch_drone", { drone_id: i });
        document.getElementById(`mgmt-kill-${i}`).onclick = () => { if (confirm(`Kill drone ${i}?`)) socket.emit("cmd_kill_drone", { drone_id: i }); };
        document.getElementById(`mgmt-takeoff-${i}`).onclick = () => socket.emit("cmd_takeoff_drone", { drone_id: i, alt: parseFloat(document.getElementById("takeoff-alt").value) || 10 });
        document.getElementById(`mgmt-land-${i}`).onclick = () => socket.emit("cmd_land_drone", { drone_id: i });
    }
}

// ── RL Mode Grid ──────────────────────────────────────

function renderRLGrid() {
    const grid = document.getElementById("rl-drone-grid");
    if (!grid) return;
    if (grid.dataset.count === String(MAX_DRONES)) {
        for (let i = 1; i <= MAX_DRONES; i++) {
            const badge = document.getElementById(`rl-badge-${i}`);
            if (!badge) continue;
            const s = latestState && latestState.drones && latestState.drones[String(i)];
            const rlOn = s && s.rl_mode;
            badge.className = `rl-badge ${rlOn ? "rl-on" : "rl-off"}`;
            badge.textContent = rlOn ? "RL" : "--";
        }
        return;
    }
    grid.innerHTML = "";
    grid.dataset.count = String(MAX_DRONES);
    for (let i = 1; i <= MAX_DRONES; i++) {
        const c = COLORS[(i - 1) % COLORS.length];
        const row = document.createElement("div");
        row.className = "rl-row";
        row.innerHTML = `
            <span style="color:${c};font-weight:600;font-family:var(--font-mono);font-size:10px;width:24px">D${i}</span>
            <span id="rl-badge-${i}" class="rl-badge rl-off">--</span>
            <button class="btn btn-rl btn-sm" id="rl-on-${i}">ON</button>
            <button class="btn btn-sm" id="rl-off-${i}">OFF</button>`;
        grid.appendChild(row);
        document.getElementById(`rl-on-${i}`).onclick = () =>
            socket.emit("cmd_rl_mode", { enable: true, target_id: i });
        document.getElementById(`rl-off-${i}`).onclick = () =>
            socket.emit("cmd_rl_mode", { enable: false, target_id: i });
    }
}

// ── Selects ────────────────────────────────────────────

function updateDroneSelects() {
    ["drone-select", "ned-drone-select"].forEach(id => {
        const sel = document.getElementById(id);
        if (!sel || sel.options.length === MAX_DRONES) return;
        const cur = sel.value;
        sel.innerHTML = "";
        for (let i = 1; i <= MAX_DRONES; i++) {
            const o = document.createElement("option");
            o.value = i; o.textContent = `D${i}`;
            sel.appendChild(o);
        }
        if (cur) sel.value = cur;
    });
}

// ── Status Table ───────────────────────────────────────

function stateClass(s) {
    if (!s) return "state-nominal";
    const u = s.toUpperCase();
    if (u === "NOMINAL") return "state-nominal";
    if (u === "DEGRADED") return "state-degraded";
    if (u === "COMMS_LOST") return "state-comms-lost";
    if (u === "COMMS_RECOVERY") return "state-comms-recovery";
    if (u === "LANDED") return "state-landed";
    return "state-emergency";
}

function updateStatusTable(data) {
    statusBody.innerHTML = "";
    const staleSet = new Set(data.stale);
    const entries = Object.entries(data.drones).sort((a, b) => parseInt(a[0]) - parseInt(b[0]));
    for (const [id, s] of entries) {
        const nid = parseInt(id);
        const blocked = data.blocked.includes(nid);
        const stale = staleSet.has(nid);
        const tr = document.createElement("tr");
        if (stale) tr.className = "stale-row";
        else if (s.failsafe_active) tr.className = "failsafe-row";
        if (blocked) tr.className += " blocked-row";
        const lid = s.leader_id || 0;
        const c = COLORS[(nid - 1) % COLORS.length];
        const rlOn = s.rl_mode || false;
        // GCS-detected stale overrides drone's self-reported state
        const displayState = stale ? "COMMS_LOST" : (s.swarm_state || "NOMINAL");
        const displayStateClass = stale ? "state-comms-lost" : stateClass(s.swarm_state);
        tr.innerHTML = `
            <td style="color:${c};font-weight:600">${stale ? "\u26A0 " : ""}D${id}</td>
            <td>${stale ? "-" : (s.mode || "?")}</td>
            <td>${(s.alt || 0).toFixed(1)}</td>
            <td>${s.battery_pct >= 0 ? s.battery_pct + "%" : "?"}</td>
            <td>${s.armed ? "YES" : "-"}</td>
            <td style="color:${s.failsafe_active ? "var(--red)" : "var(--text-muted)"}">${s.failsafe_active ? "FS" : "-"}</td>
            <td class="${displayStateClass}">${displayState}</td>
            <td>${nid === lid ? "\u2605" : ""} ${lid > 0 ? "D" + lid : "-"}</td>
            <td>${s.alive_count || 0}</td>
            <td class="${rlOn ? "rl-active" : ""}">${rlOn ? "RL" : "-"}</td>`;
        statusBody.appendChild(tr);
    }
}

// ── Elapsed ────────────────────────────────────────────

function updateElapsed(sec) {
    const m = Math.floor(sec / 60), s = Math.floor(sec % 60);
    elapsedSpan.textContent = `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}

// ── Log ────────────────────────────────────────────────

function addLog(level, msg) {
    const e = document.createElement("div");
    e.className = `log-entry log-${level.toLowerCase()}`;
    e.textContent = `[${new Date().toLocaleTimeString()}] [${level}] ${msg}`;
    logScroll.appendChild(e);
    if (logScroll.children.length > 200) logScroll.removeChild(logScroll.firstChild);
    logScroll.scrollTop = logScroll.scrollHeight;
}

// ── Panel Collapse ─────────────────────────────────────

function togglePanel(hdr) {
    const body = hdr.nextElementSibling;
    const chev = hdr.querySelector(".panel-chevron");
    body.classList.toggle("collapsed");
    chev.style.transform = body.classList.contains("collapsed") ? "rotate(-90deg)" : "";
}

// ── Failure Injection ──────────────────────────────────

function blockComms(id) {
    socket.emit("cmd_block_comms", { drone_id: id });
    const r = document.getElementById(`fail-${id}`);
    if (r) { r.querySelector(".btn-warning").disabled = true; r.querySelector(".btn-success").disabled = false; }
}

function restoreComms(id) {
    socket.emit("cmd_restore_comms", { drone_id: id });
    const r = document.getElementById(`fail-${id}`);
    if (r) { r.querySelector(".btn-warning").disabled = false; r.querySelector(".btn-success").disabled = true; }
}

// ── Button Handlers ────────────────────────────────────

document.getElementById("btn-takeoff").onclick = () => socket.emit("cmd_takeoff", { alt: parseFloat(document.getElementById("takeoff-alt").value) });
document.getElementById("btn-land").onclick = () => socket.emit("cmd_land", {});
document.getElementById("btn-launch-all").onclick = () => socket.emit("cmd_launch_all", {});
document.getElementById("btn-kill-all").onclick = () => { if (confirm("Kill ALL drones?")) socket.emit("cmd_kill_all", {}); };

document.getElementById("formation-heading").oninput = (e) => document.getElementById("heading-val").textContent = e.target.value;
document.getElementById("formation-spacing").oninput = (e) => document.getElementById("spacing-val").textContent = parseFloat(e.target.value).toFixed(1);
document.getElementById("btn-formation").onclick = () => socket.emit("cmd_formation", {
    shape: document.getElementById("formation-shape").value,
    heading_deg: parseFloat(document.getElementById("formation-heading").value),
    spacing_m: parseFloat(document.getElementById("formation-spacing").value),
});

document.getElementById("btn-waypoint").onclick = () => {
    const lat = parseFloat(document.getElementById("wp-lat").value);
    const lon = parseFloat(document.getElementById("wp-lon").value);
    if (isNaN(lat) || isNaN(lon)) { addLog("WARN", "Enter valid lat/lon or click map"); return; }
    socket.emit("cmd_waypoint", {
        drone_id: parseInt(document.getElementById("drone-select").value),
        lat, lon, alt: parseFloat(document.getElementById("wp-alt").value),
    });
};

let swarmWpMarker = null;
document.getElementById("btn-swarm-waypoint").onclick = () => {
    const lat = parseFloat(document.getElementById("swarm-wp-lat").value);
    const lon = parseFloat(document.getElementById("swarm-wp-lon").value);
    if (isNaN(lat) || isNaN(lon)) { addLog("WARN", "Enter valid lat/lon or click map"); return; }
    socket.emit("cmd_swarm_waypoint", {
        lat, lon, alt: parseFloat(document.getElementById("swarm-wp-alt").value) || 10,
    });
    if (swarmWpMarker) map.removeLayer(swarmWpMarker);
    swarmWpMarker = L.circleMarker([lat, lon], {
        radius: 12, color: "#fbbf24", fillColor: "#f59e0b", fillOpacity: 0.6, weight: 2,
    }).addTo(map).bindTooltip("Swarm WP", { permanent: true, direction: "top", className: "wp-tooltip" });
    setTimeout(() => { if (swarmWpMarker) { map.removeLayer(swarmWpMarker); swarmWpMarker = null; } }, 15000);
};

document.getElementById("btn-goto-ned").onclick = () => {
    const n = parseFloat(document.getElementById("ned-north").value);
    const e = parseFloat(document.getElementById("ned-east").value);
    if (isNaN(n) || isNaN(e)) { addLog("WARN", "Enter valid N/E offsets"); return; }
    socket.emit("cmd_waypoint_ned", {
        drone_id: parseInt(document.getElementById("ned-drone-select").value),
        north: n, east: e, alt: parseFloat(document.getElementById("ned-alt").value),
    });
};

document.getElementById("btn-clear-log").onclick = () => { logScroll.innerHTML = ""; addLog("INFO", "Log cleared"); };
document.getElementById("btn-rl-all-on").onclick = () => socket.emit("cmd_rl_mode", { enable: true, target_id: 0 });
document.getElementById("btn-rl-all-off").onclick = () => socket.emit("cmd_rl_mode", { enable: false, target_id: 0 });

document.getElementById("btn-auto-center").onclick = () => {
    autoCenter = !autoCenter;
    document.getElementById("btn-auto-center").classList.toggle("active", autoCenter);
    if (autoCenter && latestState) updateMap(latestState);
};
document.getElementById("btn-toggle-trails").onclick = () => {
    showTrails = !showTrails;
    document.getElementById("btn-toggle-trails").classList.toggle("active", showTrails);
    if (!showTrails) clearTrails();
};

document.getElementById("btn-toggle-isolation").onclick = () => {
    showIsolationZones = !showIsolationZones;
    document.getElementById("btn-toggle-isolation").classList.toggle("active", showIsolationZones);
    if (!showIsolationZones) clearIsolationZones();
    else if (latestState) updateIsolationZones(latestState);
};

// Isolation radius slider
document.getElementById("isolation-radius").oninput = (e) => {
    document.getElementById("isolation-val").textContent = parseFloat(e.target.value).toFixed(1);
};
document.getElementById("btn-set-isolation").onclick = () => {
    const radius = parseFloat(document.getElementById("isolation-radius").value);
    currentIsolationRadius = radius;
    socket.emit("cmd_set_isolation_radius", { radius_m: radius });
    // Rebuild circles with new radius
    if (showIsolationZones) {
        clearIsolationZones();
        if (latestState) updateIsolationZones(latestState);
    }
};

// ── Tab switching (bottom panels) ─────────────────────
document.querySelectorAll(".bottom-tabs").forEach(tabBar => {
    tabBar.addEventListener("click", (e) => {
        const tab = e.target.closest(".bottom-tab");
        if (!tab) return;
        const target = tab.dataset.target;
        const section = tabBar.closest(".bottom-section");
        section.querySelectorAll(".bottom-tab").forEach(t => t.classList.remove("active"));
        section.querySelectorAll(".tab-pane").forEach(p => p.classList.remove("active"));
        tab.classList.add("active");
        const pane = document.getElementById(target);
        if (pane) pane.classList.add("active");
    });
});

// ── Diagnostics ─────────────────────────────────────────

function renderDiagResults(results) {
    const container = document.getElementById("diag-results");
    container.innerHTML = "";
    let passed = 0, failed = 0, errors = 0;

    for (const r of results) {
        const card = document.createElement("div");
        card.className = `diag-card diag-${r.status}`;

        const icon = r.status === "pass" ? "\u2713" : r.status === "fail" ? "\u2717" : "\u26A0";

        if (r.status === "pass") passed++;
        else if (r.status === "fail") failed++;
        else errors++;

        let assertHtml = "";
        if (r.assertions && r.assertions.length > 0) {
            assertHtml = '<div class="diag-assertions">';
            for (const a of r.assertions) {
                const cls = a.passed ? "assert-pass" : "assert-fail";
                const ai = a.passed ? "\u2713" : "\u2717";
                assertHtml += `<div class="assert-row ${cls}">${ai} ${a.check} <span class="assert-val">${a.value}</span></div>`;
            }
            assertHtml += "</div>";
        }

        let diagHtml = "";
        if (r.test_id === "formation_geometry" && r.details && r.details.positions) {
            diagHtml = renderFormationDiagram(r.details.positions);
        }

        let errHtml = "";
        if (r.status === "error" && r.details) {
            const msg = r.details.error || r.details.message || "";
            if (msg) errHtml = `<div class="diag-error-msg">${msg}</div>`;
        }

        card.innerHTML = `
            <div class="diag-header" onclick="this.parentElement.classList.toggle('expanded')">
                <span class="diag-status diag-${r.status}">${icon}</span>
                <span class="diag-name">${r.test_name || r.test_id}</span>
                <span class="diag-time">${(r.duration_ms || 0).toFixed(0)}ms</span>
                <span class="diag-expand">&#9662;</span>
            </div>
            <div class="diag-body">${errHtml}${assertHtml}${diagHtml}</div>`;
        container.appendChild(card);
    }

    const summary = document.getElementById("diag-summary");
    const total = passed + failed + errors;
    const cls = (failed + errors > 0) ? "diag-fail" : "diag-pass";
    summary.innerHTML = `<span class="${cls}">${passed}/${total}</span>`;
}

function renderFormationDiagram(positions) {
    const keys = Object.keys(positions).filter(k => k.includes("H0") && k.includes("N5")).slice(0, 4);
    if (keys.length === 0) return "";
    let html = '<div class="diag-formations">';
    for (const key of keys) {
        const pts = positions[key];
        const sz = 90, mg = 12;
        const ns = pts.map(p => p.offset_n);
        const es = pts.map(p => p.offset_e);
        const range = Math.max(Math.max(...ns) - Math.min(...ns), Math.max(...es) - Math.min(...es), 1);
        const scale = (sz - 2 * mg) / range;
        const cn = (Math.max(...ns) + Math.min(...ns)) / 2;
        const ce = (Math.max(...es) + Math.min(...es)) / 2;
        let dots = "";
        for (const p of pts) {
            const x = mg + (p.offset_e - ce + range / 2) * scale;
            const y = mg + (-(p.offset_n - cn) + range / 2) * scale;
            const c = COLORS[(p.drone_id - 1) % COLORS.length];
            dots += `<circle cx="${x}" cy="${y}" r="3.5" fill="${c}"/>`;
            dots += `<text x="${x}" y="${y - 5}" text-anchor="middle" fill="${c}" font-size="7">D${p.drone_id}</text>`;
        }
        const label = key.split("_")[0];
        html += `<div class="formation-mini">
            <div class="formation-label">${label}</div>
            <svg width="${sz}" height="${sz}" class="formation-svg">${dots}</svg></div>`;
    }
    html += "</div>";
    return html;
}

socket.on("test_results", data => { if (data.results) renderDiagResults(data.results); });
socket.on("test_result", data => { if (data.result) renderDiagResults([data.result]); });

document.getElementById("btn-run-all-tests").onclick = () => {
    document.getElementById("diag-results").innerHTML = '<div class="diag-loading">Running tests...</div>';
    document.getElementById("diag-summary").innerHTML = "";
    socket.emit("run_all_tests", {});
};

// ── Init ───────────────────────────────────────────────

function init() {
    initMap();
    updateDroneSelects();

    // Panel collapse handlers
    document.querySelectorAll(".panel-header").forEach(h => h.addEventListener("click", () => togglePanel(h)));

    // Failure grid
    const fg = document.getElementById("failure-grid");
    for (let i = 1; i <= MAX_DRONES; i++) {
        const c = COLORS[(i - 1) % COLORS.length];
        const r = document.createElement("div");
        r.className = "failure-row";
        r.id = `fail-${i}`;
        r.innerHTML = `<span style="color:${c}">D${i}</span>
            <button class="btn btn-warning btn-sm" onclick="blockComms(${i})">Block</button>
            <button class="btn btn-success btn-sm" onclick="restoreComms(${i})" disabled>Restore</button>`;
        fg.appendChild(r);
    }

    renderDroneGrid();
    addLog("INFO", "GCS initialized, waiting for connection...");
}

init();
