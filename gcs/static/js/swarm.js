// Swarm SITL — Web GCS JavaScript
// Canvas visualization, status table, command handlers, drone management

// ── Constants ──────────────────────────────────────────
const COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00",
                "#a65628", "#f781bf", "#999999", "#66c2a5", "#fc8d62"];
const METERS_PER_DEG = 111320.0;
const LON_SCALE = 0.8;  // cos(latitude) approx for Canberra
const VIEW_RANGE = 50;  // +/- 50m

// ── State ──────────────────────────────────────────────
let refLat = null;
let refLon = null;
let latestState = null;
let processStatus = {};
let knownDrones = [];

// ── DOM references ─────────────────────────────────────
const canvas = document.getElementById("swarm-canvas");
const ctx = canvas.getContext("2d");
const statusBody = document.getElementById("status-body");
const logScroll = document.getElementById("log-scroll");
const connStatus = document.getElementById("conn-status");
const elapsedSpan = document.getElementById("elapsed");

// ── Socket.IO ──────────────────────────────────────────
const socket = io();

socket.on("connect", () => {
    connStatus.textContent = "Connected";
    connStatus.className = "connected";
    addLog("INFO", "Connected to Web GCS");
});

socket.on("disconnect", () => {
    connStatus.textContent = "Disconnected";
    connStatus.className = "disconnected";
    addLog("WARN", "Disconnected from Web GCS");
});

socket.on("state_update", (data) => {
    latestState = data;
    processStatus = data.process_status || {};
    knownDrones = data.known_drones || [];
    updateElapsed(data.elapsed_s);
    updateStatusTable(data);
    renderDroneGrid();
    updateDroneSelects();
    drawCanvas(data);
});

socket.on("alert", (data) => {
    addLog("ALERT",
        `D${data.drone_id} [${data.code}] ${data.message} -> ${data.action_taken}`);
});

socket.on("log_event", (data) => {
    addLog(data.level, data.message);
});

socket.on("drone_launched", (data) => {
    addLog("INFO", `Drone ${data.drone_id} launched (PID ${data.pid})`);
});

socket.on("drone_killed", (data) => {
    addLog("WARN", `Drone ${data.drone_id} killed`);
});

// ── Canvas ─────────────────────────────────────────────

function gpsToLocal(lat, lon) {
    if (refLat === null) return null;
    return {
        north: (lat - refLat) * METERS_PER_DEG,
        east: (lon - refLon) * METERS_PER_DEG * LON_SCALE,
    };
}

function metersToCanvas(north, east) {
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const scale = canvas.width / (VIEW_RANGE * 2);
    return {
        x: cx + east * scale,
        y: cy - north * scale,
    };
}

function canvasToMeters(cx, cy) {
    const scale = canvas.width / (VIEW_RANGE * 2);
    return {
        east: (cx - canvas.width / 2) / scale,
        north: -(cy - canvas.height / 2) / scale,
    };
}

function drawCanvas(data) {
    const drones = data.drones;
    const staleSet = new Set(data.stale);
    const blockedSet = new Set(data.blocked);

    // Background
    ctx.fillStyle = "#1a1a2e";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Grid
    ctx.strokeStyle = "rgba(255,255,255,0.07)";
    ctx.lineWidth = 1;
    for (let m = -VIEW_RANGE; m <= VIEW_RANGE; m += 10) {
        const p = metersToCanvas(m, -VIEW_RANGE);
        const q = metersToCanvas(m, VIEW_RANGE);
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.stroke();
        const r = metersToCanvas(-VIEW_RANGE, m);
        const s = metersToCanvas(VIEW_RANGE, m);
        ctx.beginPath(); ctx.moveTo(r.x, r.y); ctx.lineTo(s.x, s.y); ctx.stroke();
    }

    // Axis tick labels
    ctx.fillStyle = "rgba(255,255,255,0.25)";
    ctx.font = "9px monospace";
    for (let m = -40; m <= 40; m += 20) {
        if (m === 0) continue;
        const p = metersToCanvas(0, m);
        ctx.fillText(m + "", p.x - 6, canvas.height / 2 + 12);
        const q = metersToCanvas(m, 0);
        ctx.fillText(m + "", canvas.width / 2 + 4, q.y + 3);
    }

    // Axis labels
    ctx.fillStyle = "rgba(255,255,255,0.35)";
    ctx.font = "11px monospace";
    ctx.fillText("East (m)", canvas.width - 65, canvas.height / 2 + 25);
    ctx.save();
    ctx.translate(12, canvas.height / 2 - 20);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("North (m)", 0, 0);
    ctx.restore();

    // Origin crosshair
    const o = metersToCanvas(0, 0);
    ctx.strokeStyle = "rgba(255,255,255,0.15)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(o.x - 8, o.y); ctx.lineTo(o.x + 8, o.y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(o.x, o.y - 8); ctx.lineTo(o.x, o.y + 8); ctx.stroke();

    // Set reference from first valid drone
    let alts = [];
    let droneCount = 0;
    for (const [id, s] of Object.entries(drones)) {
        if ((s.lat !== 0 || s.lon !== 0) && refLat === null) {
            refLat = s.lat;
            refLon = s.lon;
        }
    }

    // Determine leader from first drone's leader_id
    let leaderId = 1;
    for (const [id, s] of Object.entries(drones)) {
        if (s.leader_id) { leaderId = s.leader_id; break; }
    }

    // Draw drones
    for (const [id, s] of Object.entries(drones)) {
        if (s.lat === 0 && s.lon === 0) continue;
        const local = gpsToLocal(s.lat, s.lon);
        if (!local) continue;
        const pos = metersToCanvas(local.north, local.east);
        const color = COLORS[(parseInt(id) - 1) % COLORS.length];
        const isStale = staleSet.has(parseInt(id));
        const isBlocked = blockedSet.has(parseInt(id));
        const swarmState = (s.swarm_state || "NOMINAL").toUpperCase();

        ctx.globalAlpha = isStale ? 0.25 : 1.0;

        // State ring for non-NOMINAL drones
        if (swarmState !== "NOMINAL") {
            let ringColor = "#ff7f00";
            if (swarmState === "COMMS_LOST" || swarmState.startsWith("EMERGENCY")) {
                ringColor = "#e41a1c";
            }
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, 15, 0, Math.PI * 2);
            ctx.strokeStyle = ringColor;
            ctx.lineWidth = 2.5;
            ctx.stroke();
        }

        // Drone circle
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 10, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = "rgba(255,255,255,0.3)";
        ctx.lineWidth = 1;
        ctx.stroke();

        // Heading indicator
        const hdgRad = ((s.heading || 0) - 0) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(pos.x, pos.y);
        ctx.lineTo(pos.x + Math.sin(hdgRad) * 18, pos.y - Math.cos(hdgRad) * 18);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // Leader indicator: gold star above leader
        if (parseInt(id) === leaderId) {
            ctx.fillStyle = "#ffd700";
            ctx.font = "bold 14px monospace";
            ctx.fillText("\u2605", pos.x - 5, pos.y - 16);
        }

        // Label
        ctx.fillStyle = color;
        ctx.font = "bold 11px monospace";
        ctx.fillText(`D${id} ${(s.alt || 0).toFixed(1)}m`, pos.x + 14, pos.y - 4);

        // State label for non-NOMINAL
        if (swarmState !== "NOMINAL") {
            const sc = swarmState === "DEGRADED" ? "#ff7f00" : "#e41a1c";
            ctx.fillStyle = sc;
            ctx.font = "bold 9px monospace";
            ctx.fillText(`[${swarmState}]`, pos.x + 14, pos.y + 10);
        } else if (s.failsafe_active) {
            ctx.fillStyle = "#ff4444";
            ctx.font = "bold 10px monospace";
            ctx.fillText("[FS!]", pos.x + 14, pos.y + 10);
        }

        // Blocked indicator (dashed red ring)
        if (isBlocked) {
            ctx.save();
            ctx.setLineDash([4, 4]);
            ctx.strokeStyle = "#ff4444";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, 16, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        }

        // Dead/stale marker: X over last position
        if (isStale) {
            ctx.strokeStyle = "#e41a1c";
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(pos.x - 8, pos.y - 8); ctx.lineTo(pos.x + 8, pos.y + 8);
            ctx.moveTo(pos.x + 8, pos.y - 8); ctx.lineTo(pos.x - 8, pos.y + 8);
            ctx.stroke();
            ctx.fillStyle = "rgba(255,255,255,0.4)";
            ctx.font = "9px monospace";
            ctx.fillText("[STALE]", pos.x + 14, pos.y + 20);
        }

        ctx.globalAlpha = 1.0;
        if (s.alt > 0) alts.push(s.alt);
        droneCount++;
    }

    // Title
    const avgAlt = alts.length ? (alts.reduce((a,b) => a+b, 0) / alts.length) : 0;
    ctx.fillStyle = "rgba(255,255,255,0.7)";
    ctx.font = "13px monospace";
    ctx.fillText(
        `Swarm SITL - ${droneCount} drones - avg alt ${avgAlt.toFixed(1)}m`,
        10, 20
    );
}

// ── Canvas click-to-waypoint ───────────────────────────

canvas.addEventListener("click", (e) => {
    if (refLat === null) return;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const m = canvasToMeters(cx, cy);

    // Convert meters back to GPS
    const lat = refLat + m.north / METERS_PER_DEG;
    const lon = refLon + m.east / (METERS_PER_DEG * LON_SCALE);

    document.getElementById("wp-lat").value = lat.toFixed(7);
    document.getElementById("wp-lon").value = lon.toFixed(7);
});

// ── Drone Management Grid ─────────────────────────────

function renderDroneGrid() {
    const grid = document.getElementById("drone-mgmt-grid");
    // Only rebuild if drone count changed
    if (grid.dataset.count === String(MAX_DRONES)) {
        // Just update status dots and button states
        for (let i = 1; i <= MAX_DRONES; i++) {
            const dot = document.getElementById(`mgmt-dot-${i}`);
            const status = processStatus[i] || "unmanaged";
            if (dot) {
                dot.className = `status-dot status-${status}`;
                dot.title = status;
            }
            const btnLaunch = document.getElementById(`mgmt-launch-${i}`);
            const btnKill = document.getElementById(`mgmt-kill-${i}`);
            const btnTakeoff = document.getElementById(`mgmt-takeoff-${i}`);
            const btnLand = document.getElementById(`mgmt-land-${i}`);
            if (btnLaunch) btnLaunch.disabled = (status === "running");
            if (btnKill) btnKill.disabled = (status !== "running");
            // Takeoff/land need the drone to be reporting
            const hasState = latestState && latestState.drones && latestState.drones[String(i)];
            if (btnTakeoff) btnTakeoff.disabled = !hasState;
            if (btnLand) btnLand.disabled = !hasState;
        }
        return;
    }

    // Build grid
    grid.innerHTML = "";
    grid.dataset.count = String(MAX_DRONES);
    for (let i = 1; i <= MAX_DRONES; i++) {
        const status = processStatus[i] || "unmanaged";
        const row = document.createElement("div");
        row.className = "drone-mgmt-row";
        row.id = `mgmt-row-${i}`;
        row.innerHTML = `
            <span class="status-dot status-${status}" id="mgmt-dot-${i}" title="${status}"></span>
            <span style="color:${COLORS[(i-1) % COLORS.length]}; font-weight:bold; width:28px">D${i}</span>
            <button class="btn-launch btn-sm" id="mgmt-launch-${i}" ${status === "running" ? "disabled" : ""}>Launch</button>
            <button class="btn-kill btn-sm" id="mgmt-kill-${i}" ${status !== "running" ? "disabled" : ""}>Kill</button>
            <button class="btn-sm" id="mgmt-takeoff-${i}" disabled>Takeoff</button>
            <button class="btn-land-single btn-sm" id="mgmt-land-${i}" disabled>Land</button>
        `;
        grid.appendChild(row);

        // Bind events
        document.getElementById(`mgmt-launch-${i}`).addEventListener("click", () => {
            socket.emit("cmd_launch_drone", { drone_id: i });
        });
        document.getElementById(`mgmt-kill-${i}`).addEventListener("click", () => {
            if (!confirm(`Kill drone ${i}?`)) return;
            socket.emit("cmd_kill_drone", { drone_id: i });
        });
        document.getElementById(`mgmt-takeoff-${i}`).addEventListener("click", () => {
            const alt = parseFloat(document.getElementById("takeoff-alt").value) || 10;
            socket.emit("cmd_takeoff_drone", { drone_id: i, alt: alt });
        });
        document.getElementById(`mgmt-land-${i}`).addEventListener("click", () => {
            socket.emit("cmd_land_drone", { drone_id: i });
        });
    }
}

// ── Dynamic drone selects ─────────────────────────────

function updateDroneSelects() {
    updateSelectOptions("drone-select");
    updateSelectOptions("ned-drone-select");
}

function updateSelectOptions(selectId) {
    const sel = document.getElementById(selectId);
    if (!sel) return;
    const current = sel.value;

    // Build list: all drones 1..MAX, mark known ones
    const needed = [];
    for (let i = 1; i <= MAX_DRONES; i++) {
        needed.push(i);
    }

    // Only rebuild if options count changed
    if (sel.options.length === needed.length) return;

    sel.innerHTML = "";
    for (const id of needed) {
        const opt = document.createElement("option");
        opt.value = id;
        opt.textContent = `Drone ${id}`;
        sel.appendChild(opt);
    }
    if (current) sel.value = current;
}

// ── Status Table ───────────────────────────────────────

function stateClass(swarmState) {
    if (!swarmState) return "state-nominal";
    const s = swarmState.toUpperCase();
    if (s === "NOMINAL") return "state-nominal";
    if (s === "DEGRADED") return "state-degraded";
    if (s === "COMMS_LOST") return "state-comms-lost";
    if (s === "COMMS_RECOVERY") return "state-comms-recovery";
    if (s === "LANDED") return "state-landed";
    return "state-emergency";
}

function updateStatusTable(data) {
    statusBody.innerHTML = "";
    const entries = Object.entries(data.drones).sort((a,b) => parseInt(a[0]) - parseInt(b[0]));
    for (const [id, s] of entries) {
        const isBlocked = data.blocked.includes(parseInt(id));
        const tr = document.createElement("tr");
        if (s.failsafe_active) tr.className = "failsafe-row";
        if (isBlocked) tr.className += " blocked-row";
        const sc = stateClass(s.swarm_state);
        const leaderId = s.leader_id || 1;
        const isLeader = parseInt(id) === leaderId;
        tr.innerHTML = `
            <td style="color:${COLORS[(parseInt(id)-1) % COLORS.length]}; font-weight:bold">D${id}</td>
            <td>${s.mode || "?"}</td>
            <td>${(s.alt || 0).toFixed(1)}</td>
            <td>${s.battery_pct >= 0 ? s.battery_pct + "%" : "?"}</td>
            <td>${s.armed ? "YES" : "-"}</td>
            <td style="color:${s.failsafe_active ? '#e41a1c' : '#333'}">${s.failsafe_active ? "ACTIVE" : "-"}</td>
            <td class="${sc}">${s.swarm_state || "NOMINAL"}</td>
            <td>${isLeader ? "&#9733;" : ""} D${leaderId}</td>
            <td>${s.alive_count || 0}</td>
        `;
        statusBody.appendChild(tr);
    }
}

// ── Elapsed Time ───────────────────────────────────────

function updateElapsed(seconds) {
    const min = Math.floor(seconds / 60);
    const sec = Math.floor(seconds % 60);
    elapsedSpan.textContent =
        `${String(min).padStart(2,"0")}:${String(sec).padStart(2,"0")}`;
}

// ── Event Log ──────────────────────────────────────────

function addLog(level, message) {
    const entry = document.createElement("div");
    entry.className = `log-entry log-${level.toLowerCase()}`;
    const now = new Date().toLocaleTimeString();
    entry.textContent = `[${now}] [${level}] ${message}`;
    logScroll.appendChild(entry);
    if (logScroll.children.length > 200) {
        logScroll.removeChild(logScroll.firstChild);
    }
    logScroll.scrollTop = logScroll.scrollHeight;
}

// ── Button Handlers ────────────────────────────────────

// Global commands
document.getElementById("btn-takeoff").addEventListener("click", () => {
    const alt = parseFloat(document.getElementById("takeoff-alt").value);
    socket.emit("cmd_takeoff", { alt: alt });
});

document.getElementById("btn-land").addEventListener("click", () => {
    socket.emit("cmd_land", {});
});

// Launch All / Kill All
document.getElementById("btn-launch-all").addEventListener("click", () => {
    socket.emit("cmd_launch_all", {});
});

document.getElementById("btn-kill-all").addEventListener("click", () => {
    if (!confirm("Kill ALL drone processes?")) return;
    socket.emit("cmd_kill_all", {});
});

// Formation sliders
document.getElementById("formation-heading").addEventListener("input", (e) => {
    document.getElementById("heading-val").textContent = e.target.value;
});
document.getElementById("formation-spacing").addEventListener("input", (e) => {
    document.getElementById("spacing-val").textContent = parseFloat(e.target.value).toFixed(1);
});
document.getElementById("btn-formation").addEventListener("click", () => {
    socket.emit("cmd_formation", {
        shape: document.getElementById("formation-shape").value,
        heading_deg: parseFloat(document.getElementById("formation-heading").value),
        spacing_m: parseFloat(document.getElementById("formation-spacing").value),
    });
});

// GPS Waypoint
document.getElementById("btn-waypoint").addEventListener("click", () => {
    const droneId = parseInt(document.getElementById("drone-select").value);
    const lat = parseFloat(document.getElementById("wp-lat").value);
    const lon = parseFloat(document.getElementById("wp-lon").value);
    const alt = parseFloat(document.getElementById("wp-alt").value);
    if (isNaN(lat) || isNaN(lon)) {
        addLog("WARN", "Enter valid lat/lon or click on canvas");
        return;
    }
    socket.emit("cmd_waypoint", { drone_id: droneId, lat: lat, lon: lon, alt: alt });
});

// NED Waypoint
document.getElementById("btn-goto-ned").addEventListener("click", () => {
    const droneId = parseInt(document.getElementById("ned-drone-select").value);
    const north = parseFloat(document.getElementById("ned-north").value);
    const east = parseFloat(document.getElementById("ned-east").value);
    const alt = parseFloat(document.getElementById("ned-alt").value);
    if (isNaN(north) || isNaN(east)) {
        addLog("WARN", "Enter valid North and East offsets");
        return;
    }
    socket.emit("cmd_waypoint_ned", { drone_id: droneId, north: north, east: east, alt: alt });
});

// ── Failure injection ──────────────────────────────────

function blockComms(droneId) {
    socket.emit("cmd_block_comms", { drone_id: droneId });
    const row = document.getElementById(`failure-row-${droneId}`);
    if (row) {
        row.querySelector(".btn-block").disabled = true;
        row.querySelector(".btn-restore").disabled = false;
    }
}

function restoreComms(droneId) {
    socket.emit("cmd_restore_comms", { drone_id: droneId });
    const row = document.getElementById(`failure-row-${droneId}`);
    if (row) {
        row.querySelector(".btn-block").disabled = false;
        row.querySelector(".btn-restore").disabled = true;
    }
}

// ── Initialization ─────────────────────────────────────

function init() {
    // Drone selectors
    updateDroneSelects();

    // Failure grid
    const grid = document.getElementById("failure-grid");
    for (let i = 1; i <= MAX_DRONES; i++) {
        const row = document.createElement("div");
        row.className = "failure-row";
        row.id = `failure-row-${i}`;
        row.innerHTML = `
            <span style="color:${COLORS[(i-1) % COLORS.length]}">D${i}</span>
            <button class="btn-block" onclick="blockComms(${i})">Block</button>
            <button class="btn-restore" onclick="restoreComms(${i})" disabled>Restore</button>
        `;
        grid.appendChild(row);
    }

    // Drone management grid (initial render)
    renderDroneGrid();

    // Draw empty canvas
    ctx.fillStyle = "#1a1a2e";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "rgba(255,255,255,0.3)";
    ctx.font = "14px monospace";
    ctx.fillText("Waiting for drone data...", canvas.width/2 - 100, canvas.height/2);

    addLog("INFO", "Web GCS initialized, waiting for connection...");
}

init();
