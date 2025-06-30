// Prevent all forms from submitting and refreshing the page
document.addEventListener('submit', function(e) {
    e.preventDefault();
});

let data = null;

async function loadTelemetry() {
    const res = await fetch('telemetry.json', { cache: "no-store" });
    data = await res.json();
}

// Function to update dashboard with telemetry data
function updateDashboardWithTelemetry() {
    // Update system status
    const systemStatusIndicator = document.getElementById('systemStatusIndicator');
    const systemStatusText = document.getElementById('systemStatusText');
    
    if (data.system_status === 'OFFLINE') {
        systemStatusIndicator.className = 'status-indicator status-error';
        systemStatusText.textContent = 'OFFLINE';
        systemStatusText.style.color = '#ff4444';
    } else {
        systemStatusIndicator.className = 'status-indicator status-online';
        systemStatusText.textContent = 'ONLINE';
        systemStatusText.style.color = '#00ff88';
    }
    
    // Update timestamps
    document.getElementById('lastUpdate').textContent = data.timestamp + ' UTC';
    document.getElementById('lastTransmission').textContent = data.timestamp + ' UTC';
    
    // Update ADCS data
    document.getElementById('pitch').textContent = (data.adcs.pitch >= 0 ? '+' : '') + data.adcs.pitch + '°';
    document.getElementById('roll').textContent = (data.adcs.roll >= 0 ? '+' : '') + data.adcs.roll + '°';
    document.getElementById('yaw').textContent = '+' + data.adcs.yaw + '°';
    
    // Update GPS data
    document.getElementById('latitude').textContent = data.adcs.gps.latitude + '° N';
    document.getElementById('longitude').textContent = data.adcs.gps.longitude + '° E';
    document.getElementById('altitude').textContent = data.adcs.gps.altitude + ' km';

    // Update map with cubesat location
    updateMapWithTelemetry();

    // Update light sensors
    document.getElementById('lightSensorF').textContent = data.adcs.light_sensors.face_a;
    document.getElementById('lightSensorB').textContent = data.adcs.light_sensors.face_b;
    document.getElementById('lightSensorL').textContent = data.adcs.light_sensors.face_c ?? 0;
    document.getElementById('lightSensorR').textContent = data.adcs.light_sensors.face_d ?? 0;
    
    // Update EPS data
    document.getElementById('batteryVoltage').textContent = data.eps.battery_voltage + 'V';
    document.getElementById('batteryPercent').textContent = data.eps.battery_percentage + '%';
    document.getElementById('batteryPercentage').textContent = data.eps.battery_percentage + '%';
    document.getElementById('current').textContent = data.eps.current + 'A';
    document.getElementById('solarPower').textContent = data.eps.solar_power + 'W';
    document.getElementById('temperature').textContent = data.eps.temperature + '°C';
    document.getElementById('systemTemp').textContent = data.adcs.temperature + '°C';

    // EPS Charging Status
    const chargingStatusElem = document.getElementById('chargingStatus');
    chargingStatusElem.textContent = data.eps.charging_status;
    chargingStatusElem.style.color = data.eps.charging_status.toUpperCase().includes('NOT') ? '#ff6666' : '#00ff88';

    // Update battery progress bars
    const batteryBar = document.getElementById('batteryBar');
    const batteryProgress = document.getElementById('batteryProgress');
    batteryBar.style.width = data.eps.battery_percentage + '%';
    batteryProgress.style.width = data.eps.battery_percentage + '%';

    // --- BATTERY WARNING LOGIC ---
    let batteryColor, batteryTextColor, batteryWarnText = '', batteryWarnColor = '', showBatteryWarn = false;
    if (data.eps.battery_percentage <= 20) {
        batteryColor = '#ff4444';
        batteryTextColor = '#ff4444';
        batteryWarnText = '⚠️ Battery critically low!';
        batteryWarnColor = '#ff4444';
        showBatteryWarn = true;
    } else if (data.eps.battery_percentage < 25) {
        batteryColor = '#ffaa00';
        batteryTextColor = '#ffaa00';
        batteryWarnText = '⚠️ Battery low!';
        batteryWarnColor = '#ffaa00';
        showBatteryWarn = true;
    } else {
        batteryColor = '#00ff88';
        batteryTextColor = '#00ff88';
        showBatteryWarn = false;
    }
    batteryBar.style.backgroundColor = batteryColor;
    batteryProgress.style.background = `linear-gradient(90deg, ${batteryColor}, ${batteryColor})`;

    // Set battery percentage text color
    document.getElementById('batteryPercent').style.color = batteryTextColor;
    document.getElementById('batteryPercentage').style.color = batteryTextColor;

    // --- DASHBOARD MINI-GRAPH BATTERY WARNING (below batteryBar, not inside) ---
    const batteryBarElem = document.getElementById('batteryBar');
    if (batteryBarElem) {
        // Find the mini-chart container (parent of batteryBar)
        const miniChart = batteryBarElem.parentNode;
        // Place warning after the mini-chart (not inside)
        let dashboardBatteryWarning = document.getElementById('dashboardBatteryWarning');
        if (!dashboardBatteryWarning || dashboardBatteryWarning.parentNode !== miniChart.parentNode) {
            if (dashboardBatteryWarning) dashboardBatteryWarning.remove();
            dashboardBatteryWarning = document.createElement('div');
            dashboardBatteryWarning.id = 'dashboardBatteryWarning';
            dashboardBatteryWarning.style.fontWeight = 'bold';
            dashboardBatteryWarning.style.marginTop = '5px';
            dashboardBatteryWarning.style.fontSize = '0.95em';
            miniChart.parentNode.insertBefore(dashboardBatteryWarning, miniChart.nextSibling);
        }
        if (showBatteryWarn) {
            dashboardBatteryWarning.textContent = batteryWarnText;
            dashboardBatteryWarning.style.color = batteryWarnColor;
            dashboardBatteryWarning.style.display = 'block';
        } else {
            dashboardBatteryWarning.textContent = '';
            dashboardBatteryWarning.style.display = 'none';
        }
    }

    // --- EPS PANEL BATTERY WARNING (below EPS bar, not inside) ---
    if (batteryProgress) {
        // Find the .progress-bar container (parent of batteryProgress)
        const progressBar = batteryProgress.parentNode;
        // Place warning after the progress bar (not inside)
        let epsBatteryWarning = document.getElementById('epsBatteryWarning');
        if (!epsBatteryWarning || epsBatteryWarning.parentNode !== progressBar.parentNode) {
            if (epsBatteryWarning) epsBatteryWarning.remove();
            epsBatteryWarning = document.createElement('div');
            epsBatteryWarning.id = 'epsBatteryWarning';
            epsBatteryWarning.style.fontWeight = 'bold';
            epsBatteryWarning.style.marginTop = '5px';
            progressBar.parentNode.insertBefore(epsBatteryWarning, progressBar.nextSibling);
        }
        if (showBatteryWarn) {
            epsBatteryWarning.textContent = batteryWarnText;
            epsBatteryWarning.style.color = batteryWarnColor;
            epsBatteryWarning.style.display = 'block';
        } else {
            epsBatteryWarning.textContent = '';
            epsBatteryWarning.style.display = 'none';
        }
    }

    // Update temperature bar (top mini-graph) to be solid color, green/yellow/red based on system temp
    const systemTempPercent = Math.round(((data.adcs.temperature - 15) / 70) * 100);
    const tempLine = document.querySelector('.temp-line');
    let tempBarColor = '#00ff88';
    if (data.adcs.temperature > 70) {
        tempBarColor = '#ff6666';
    } else if (data.adcs.temperature >= 60) {
        tempBarColor = '#ffaa00';
    }
    if (tempLine) {
        tempLine.style.width = systemTempPercent + '%';
        tempLine.style.background = tempBarColor;
        tempLine.style.animation = 'none';
    }
    // Show/hide temperature warning for system temperature
    let tempWarning = document.getElementById('tempWarning');
    if (!tempWarning && tempLine && tempLine.parentNode) {
        tempWarning = document.createElement('div');
        tempWarning.id = 'tempWarning';
        tempWarning.style.fontWeight = 'bold';
        tempWarning.style.marginTop = '5px';
        tempLine.parentNode.parentNode.appendChild(tempWarning);
    }
    if (tempWarning) {
        if (data.adcs.temperature > 70) {
            tempWarning.textContent = '🔥 System temperature CRITICAL!';
            tempWarning.style.color = '#ff4444';
            tempWarning.style.display = 'block';
        } else if (data.adcs.temperature >= 60) {
            tempWarning.textContent = '⚠️ System temperature elevated!';
            tempWarning.style.color = '#ffaa00';
            tempWarning.style.display = 'block';
        } else {
            tempWarning.textContent = '';
            tempWarning.style.display = 'none';
        }
    }

    // EPS Battery Temperature Bar (in EPS panel)
    let epsTempBar = document.getElementById('epsTempBar');
    if (!epsTempBar) {
        // Find the EPS panel and insert the bar after the temperature value
        const epsPanel = document.getElementById('temperature')?.parentNode;
        if (epsPanel) {
            epsTempBar = document.createElement('div');
            epsTempBar.id = 'epsTempBar';
            epsTempBar.className = 'progress-bar';
            epsTempBar.innerHTML = '<div id="epsTempFill" class="progress-fill"></div>';
            epsPanel.parentNode.appendChild(epsTempBar);

            // Add warning below bar
            const epsTempWarn = document.createElement('div');
            epsTempWarn.id = 'epsTempWarning';
            epsTempWarn.style.color = '#ff6666';
            epsTempWarn.style.fontWeight = 'bold';
            epsTempWarn.style.marginTop = '5px';
            epsTempBar.parentNode.appendChild(epsTempWarn);
        }
    }
    // --- EPS BATTERY TEMPERATURE COLOR & WARNING LOGIC ---
    const epsTempFill = document.getElementById('epsTempFill');
    let epsTempBarColor = '#00ff88'; // green
    let epsTempWarnText = '';
    let epsTempWarnColor = '';
    let epsTempWarnDisplay = false;
    if (data.eps.temperature > 70) {
        epsTempBarColor = '#ff4444';
        epsTempWarnText = '🔥 Battery temperature CRITICAL!';
        epsTempWarnColor = '#ff4444';
        epsTempWarnDisplay = true;
    } else if (data.eps.temperature >= 60) {
        epsTempBarColor = '#ffaa00';
        epsTempWarnText = '⚠️ Battery temperature elevated!';
        epsTempWarnColor = '#ffaa00';
        epsTempWarnDisplay = true;
    }
    const epsTempPercent = Math.round(((data.eps.temperature - 15) / 70) * 100);
    if (epsTempFill) {
        epsTempFill.style.width = epsTempPercent + '%';
        epsTempFill.style.background = epsTempBarColor;
    }
    let epsTempWarning = document.getElementById('epsTempWarning');
    if (epsTempWarning) {
        if (epsTempWarnDisplay) {
            epsTempWarning.textContent = epsTempWarnText;
            epsTempWarning.style.color = epsTempWarnColor;
            epsTempWarning.style.display = 'block';
        } else {
            epsTempWarning.textContent = '';
            epsTempWarning.style.display = 'none';
        }
    }

    // Update OBC data
    document.getElementById('storageUsage').textContent = data.obc.storage_usage + '%';
    document.getElementById('storageProgress').style.width = data.obc.storage_usage + '%';

    // Storage bar color and warning
    const storageProgress = document.getElementById('storageProgress');
    let storageColor = '#00ff88';
    let showStorageWarn = false;
    let storageWarnText = '';
    let storageWarnColor = '';
    if (data.obc.storage_usage > 85) {
        storageColor = '#ff6666';
        storageWarnText = '⚠️ Storage almost full! Please clear space.';
        storageWarnColor = '#ff6666';
        showStorageWarn = true;
    } else if (data.obc.storage_usage >= 80) {
        storageColor = '#ffaa00';
        storageWarnText = '⚠️ Storage space low!';
        storageWarnColor = '#ffaa00';
        showStorageWarn = true;
    }
    storageProgress.style.background = `linear-gradient(90deg, ${storageColor}, ${storageColor})`;

    // Set storage usage value color to match bar
    document.getElementById('storageUsage').style.color = storageColor;

    // Show/hide storage warning (styled like battery warnings, below the bar)
    let storageWarning = document.getElementById('storageWarning');
    if (storageProgress) {
        const progressBar = storageProgress.parentNode;
        if (!storageWarning || storageWarning.parentNode !== progressBar.parentNode) {
            if (storageWarning) storageWarning.remove();
            storageWarning = document.createElement('div');
            storageWarning.id = 'storageWarning';
            storageWarning.style.fontWeight = 'bold';
            storageWarning.style.marginTop = '5px';
            progressBar.parentNode.insertBefore(storageWarning, progressBar.nextSibling);
        }
        if (showStorageWarn) {
            storageWarning.textContent = storageWarnText;
            storageWarning.style.color = storageWarnColor;
            storageWarning.style.display = 'block';
        } else {
            storageWarning.textContent = '';
            storageWarning.style.display = 'none';
        }
    }

    document.getElementById('commandQueue').textContent = data.obc.command_queue;

    // OBC Data Logging, RTC Sync, I2C Status
    const dataLoggingElem = document.getElementById('dataLogging');
    dataLoggingElem.textContent = data.obc.data_logging;
    dataLoggingElem.style.color = data.obc.data_logging.toUpperCase().includes('NOT') ? '#ff4444' : '#00ff88';

    const rtcSyncElem = document.getElementById('rtcSync');
    rtcSyncElem.textContent = data.obc.rtc_sync;
    rtcSyncElem.style.color = data.obc.rtc_sync.toUpperCase().includes('NOT') ? '#ff4444' : '#00ff88';

    const i2cStatusElem = document.getElementById('i2cStatus');
    i2cStatusElem.textContent = data.obc.i2c_status;
    i2cStatusElem.style.color = data.obc.i2c_status.toUpperCase().includes('NOT') ? '#ff4444' : '#00ff88';

    // Update COMMS data (with IDs restored)
    const rfLinkElem = document.getElementById('rfLink');
    const rfLinkBars = document.getElementById('rfLinkBars');
    const rfLinkValue = data.comms.rf_link.toUpperCase();

    rfLinkElem.textContent = data.comms.rf_link;

    // Set color for RF Link text and bars
    let rfColor = '#00ff88'; // green
    let barColors = ['#00ff88', '#00ff88', '#00ff88'];
    if (rfLinkValue === 'STRONG') {
        rfColor = '#00ff88';
        barColors = ['#00ff88', '#00ff88', '#00ff88'];
    } else if (rfLinkValue === 'MEDIUM') {
        rfColor = '#ffaa00';
        barColors = ['#ffaa00', '#ffaa00', '#444'];
    } else if (rfLinkValue === 'WEAK') {
        rfColor = '#ff4444';
        barColors = ['#ff4444', '#444', '#444'];
    }
    rfLinkElem.style.color = rfColor;

    // Render bars
    if (rfLinkBars) {
        rfLinkBars.innerHTML = `
            <span class="rf-bar" style="background:${barColors[0]};opacity:${barColors[0] !== '#444' ? 1 : 0.3};"></span>
            <span class="rf-bar" style="background:${barColors[1]};opacity:${barColors[1] !== '#444' ? 1 : 0.3};"></span>
            <span class="rf-bar" style="background:${barColors[2]};opacity:${barColors[2] !== '#444' ? 1 : 0.3};"></span>
        `;
    }

    document.getElementById('packetsSent').textContent = data.comms.packets_sent.toLocaleString();
    document.getElementById('packetsReceived').textContent = data.comms.packets_received.toLocaleString();
    document.getElementById('signalStrength').textContent = data.comms.signal_strength + ' dBm';
    document.getElementById('lastCommand').textContent = `Last: "${data.comms.last_command}" - ${data.timestamp.split(' ')[1]}`;
    // Optionally update transmissionQueue if you want to make it dynamic
    // document.getElementById('transmissionQueue').innerHTML = ...;

    // Update Payload data
    const payloadStatus = document.getElementById('payloadStatus');
    payloadStatus.textContent = data.payload.status;
    // Set color: red if OFFLINE, yellow if CAPTURING, green otherwise
    if (data.payload.status === 'OFFLINE') {
        payloadStatus.style.color = '#ff4444';
    } else if (data.payload.status === 'CAPTURING') {
        payloadStatus.style.color = '#ffaa00';
    } else {
        payloadStatus.style.color = '#00ff88';
    }
    
    document.getElementById('imagesCount').textContent = data.payload.images_today;
    document.getElementById('imageSize').textContent = data.payload.last_image_size + ' MB';
    document.getElementById('aiResult').textContent = data.payload.ai_classification.result;
    document.getElementById('aiConfidence').textContent = data.payload.ai_classification.confidence + '%';
    
    // Update subsystem status indicators
    const adcsStatus = document.getElementById('adcsStatus');
    const epsStatus = document.getElementById('epsStatus');
    const obcStatus = document.getElementById('obcStatus');
    const commsStatus = document.getElementById('commsStatus');
    const payloadSubsystemStatus = document.getElementById('payloadSubsystemStatus');

    if (data.system_status === 'OFFLINE') {
        adcsStatus.className = 'status-indicator status-error';
        epsStatus.className = 'status-indicator status-error';
        obcStatus.className = 'status-indicator status-error';
        commsStatus.className = 'status-indicator status-error';
        payloadSubsystemStatus.className = 'status-indicator status-error';
    } else {
        adcsStatus.className = 'status-indicator ' + (data.adcs.status === 'ONLINE' ? 'status-online' : 'status-error');
        epsStatus.className = 'status-indicator ' + (data.eps.status === 'ONLINE' ? 'status-online' : 'status-error');
        obcStatus.className = 'status-indicator ' + (data.obc.status === 'ONLINE' ? 'status-online' : 'status-error');
        commsStatus.className = 'status-indicator ' + (data.comms.status === 'ONLINE' ? 'status-online' : 'status-error');
        payloadSubsystemStatus.className = 'status-indicator ' + (data.payload.status === 'ONLINE' ?
            (data.payload.payload_status === 'CAPTURING' ? 'status-warning' : 'status-online') : 'status-error');
    }
}

// --- MAP LOGIC (Leaflet.js) ---
let map, cubesatMarker;
function updateMapWithTelemetry() {
    // Ensure Leaflet is loaded and the map container exists
    const mapContainer = document.getElementById('map');
    if (!mapContainer) return;

    // Set map container size to fit the placeholder (if not already styled)
    mapContainer.style.width = '100%';
    mapContainer.style.height = '100%';

    // Initialize map only once
    if (!map) {
        map = L.map('map', {
            zoomControl: true,
            attributionControl: false,
            dragging: true,
            scrollWheelZoom: true,
            doubleClickZoom: true,
            boxZoom: true,
            keyboard: true,
            tap: true,
        }).setView([data.adcs.gps.latitude, data.adcs.gps.longitude], 5);

        // Use Esri World Imagery for satellite view
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 17,
            minZoom: 2,
            attribution: 'Tiles &copy; Esri'
        }).addTo(map);

        // Add marker for cubesat
        cubesatMarker = L.marker([data.adcs.gps.latitude, data.adcs.gps.longitude]).addTo(map);
    } else {
        // Update marker position and map center
        cubesatMarker.setLatLng([data.adcs.gps.latitude, data.adcs.gps.longitude]);
        map.setView([data.adcs.gps.latitude, data.adcs.gps.longitude]);
    }
}

async function registerCommand(command, status) {
    try {
        const res = await fetch('http://localhost:3001/telemetry.json', { cache: "no-store" });
        const telemetry = await res.json();
        telemetry.last_command_button = command;
        telemetry.last_command_time = new Date().toISOString();
        telemetry.command_status = status;
        if (telemetry.comms) telemetry.comms.last_command = command;
        // Use full backend URL for POST
        await fetch('http://localhost:3001/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(telemetry)
        });
    } catch (e) {
        // Optionally handle error
    }
}

function sendCommand(command) {
    const statusElement = document.getElementById('commandStatus');
    let feedbackElem = document.getElementById('commandFeedback');
    if (!feedbackElem) {
        feedbackElem = document.createElement('div');
        feedbackElem.id = 'commandFeedback';
        feedbackElem.style.fontSize = '0.9em';
        feedbackElem.style.marginTop = '6px';
        feedbackElem.style.color = '#ccc';
        statusElement.parentNode.insertBefore(feedbackElem, statusElement.nextSibling);
    }

    // Set status to "Sending Command"
    registerCommand(command, "Sending Command");

    // Show custom sentences and colors for each button immediately
    let msg = '';
    let color = '';
    switch (command) {
        case 'RESET':
            msg = 'System is resetting...';
            color = '#ffd166';
            break;
        case 'CAPTURE':
            msg = 'Capturing image...';
            color = '#3ec6ff';
            break;
        case 'SYNC':
            msg = 'Clock is synchronizing...';
            color = '#43e97b';
            break;
        case 'STATUS_CHECK':
            msg = 'Checking system status...';
            color = '#ffd166';
            break;
        default:
            msg = `Sending ${command} command...`;
            color = '#ccc';
    }
    statusElement.textContent = msg;
    statusElement.style.color = color;

    // Set status to "Waiting" after a short delay
    setTimeout(() => {
        registerCommand(command, "Waiting");
    }, 200);

    setTimeout(() => {
        let doneMsg = '';
        let doneColor = '#00ff88';
        let success = true;
        switch (command) {
            case 'RESET':
                doneMsg = 'System reset complete!';
                doneColor = '#43e97b';
                break;
            case 'CAPTURE':
                doneMsg = 'Image captured successfully!';
                doneColor = '#3ec6ff';
                break;
            case 'SYNC':
                doneMsg = 'Clock synchronized!';
                doneColor = '#43e97b';
                break;
            case 'STATUS_CHECK':
                doneMsg = 'Status check complete! All systems nominal.';
                doneColor = '#43e97b';
                break;
            default:
                doneMsg = `${command} command sent successfully!`;
        }
        statusElement.textContent = doneMsg;
        statusElement.style.color = doneColor;
        feedbackElem.textContent = '';
        // Set status to "Command Success"
        registerCommand(command, "Command Success");
    }, 1200);
}

function emergencyStop() {
    if (confirm('Are you sure you want to execute EMERGENCY STOP?')) {
        const statusElement = document.getElementById('commandStatus');
        statusElement.textContent = 'EMERGENCY STOP INITIATED!';
        statusElement.style.color = '#ff6b6b';
        let feedbackElem = document.getElementById('commandFeedback');
        if (!feedbackElem) {
            feedbackElem = document.createElement('div');
            feedbackElem.id = 'commandFeedback';
            feedbackElem.style.fontSize = '0.9em';
            feedbackElem.style.marginTop = '6px';
            feedbackElem.style.color = '#ccc';
            statusElement.parentNode.insertBefore(feedbackElem, statusElement.nextSibling);
        }
        // Set status to "Sending Command"
        registerCommand('EMERGENCY_STOP', "Sending Command");
        setTimeout(() => {
            // Set status to "Waiting"
            registerCommand('EMERGENCY_STOP', "Waiting");
        }, 200);
        setTimeout(() => {
            statusElement.textContent = 'EMERGENCY STOP ACTIVE';
            statusElement.style.color = '#ff4444';
            feedbackElem.textContent = '';
            // Set status to "Command Success"
            registerCommand('EMERGENCY_STOP', "Command Success");
        }, 1500);
    }
}

function addLogEntry(message, type) {
    const logConsole = document.getElementById('logConsole');
    const now = new Date();
    const timestamp = now.toTimeString().substring(0, 8);

    const logEntry = document.createElement('div');
    logEntry.className = `log-entry log-${type}`;
    logEntry.textContent = `[${timestamp}] ${type.toUpperCase()}: ${message}`;

    logConsole.appendChild(logEntry);
    logConsole.scrollTop = logConsole.scrollHeight;

    // Keep only last 20 entries
    while (logConsole.children.length > 20) {
        logConsole.removeChild(logConsole.firstChild);
    }
}

// --- TELEMETRY CHARTS (Chart.js Dummy Data) ---
function renderTelemetryCharts() {
    // Battery Percentage Chart
    const batteryVoltageCtx = document.getElementById('batteryVoltageChart').getContext('2d');
    new Chart(batteryVoltageCtx, {
        type: 'line',
        data: {
            labels: Array.from({length: 24}, (_, i) => `${i}:00`),
            datasets: [{
                label: 'Battery (%)',
                data: [100,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76],
                borderColor: '#00d4ff',
                backgroundColor: 'rgba(0,212,255,0.1)',
                tension: 0.3,
                pointRadius: 0
            }]
        },
        options: {
            responsive: true,
            plugins: {
                legend: { display: false },
                title: { display: false },
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Time (HH:00)',
                        color: '#ccc',
                        font: { weight: 'bold', size: 15 }
                    },
                    ticks: {
                        color: '#ccc',
                        font: { size: 14, weight: 'bold' },
                        maxTicksLimit: 12,
                        autoSkip: false,
                        callback: function(value, index, values) {
                            return index % 2 === 0 ? this.getLabelForValue(value) : '';
                        }
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                },
                y: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Battery (%)',
                        color: '#00d4ff',
                        font: { weight: 'bold', size: 15 }
                    },
                    min: 70,
                    max: 100,
                    ticks: {
                        color: '#00d4ff',
                        font: { size: 14, weight: 'bold' },
                        stepSize: 5,
                        callback: v => v + '%'
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                }
            }
        }
    });

    // Temperature Chart
    const temperatureCtx = document.getElementById('temperatureChart').getContext('2d');
    new Chart(temperatureCtx, {
        type: 'line',
        data: {
            labels: Array.from({length: 24}, (_, i) => `${i}:00`),
            datasets: [{
                label: 'Temp (°C)',
                data: [23,22,22,21,21,20,20,21,22,23,24,25,26,27,28,29,28,27,26,25,24,23,23,23],
                borderColor: '#ff6b6b',
                backgroundColor: 'rgba(255,107,107,0.1)',
                tension: 0.3,
                pointRadius: 0
            }]
        },
        options: {
            responsive: true,
            plugins: {
                legend: { display: false },
                title: { display: false },
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Time (HH:00)',
                        color: '#ccc',
                        font: { weight: 'bold', size: 15 }
                    },
                    ticks: {
                        color: '#ccc',
                        font: { size: 14, weight: 'bold' },
                        maxTicksLimit: 12,
                        autoSkip: false,
                        callback: function(value, index, values) {
                            return index % 2 === 0 ? this.getLabelForValue(value) : '';
                        }
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                },
                y: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Temperature (°C)',
                        color: '#ff6b6b',
                        font: { weight: 'bold', size: 15 }
                    },
                    min: 18,
                    max: 30,
                    ticks: {
                        color: '#ff6b6b',
                        font: { size: 14, weight: 'bold' },
                        stepSize: 2,
                        callback: v => v.toFixed(0)
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                }
            }
        }
    });

    // Packet Success Rate Chart
    const packetSuccessCtx = document.getElementById('packetSuccessChart').getContext('2d');
    new Chart(packetSuccessCtx, {
        type: 'line',
        data: {
            labels: Array.from({length: 24}, (_, i) => `${i}:00`),
            datasets: [{
                label: 'Success (%)',
                data: [96,97,98,97,96,95,94,95,96,97,98,99,98,97,96,95,94,95,96,97,98,99,98,97],
                borderColor: '#00ff88',
                backgroundColor: 'rgba(0,255,136,0.1)',
                tension: 0.3,
                pointRadius: 0
            }]
        },
        options: {
            responsive: true,
            plugins: {
                legend: { display: false },
                title: { display: false },
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Time (HH:00)',
                        color: '#ccc',
                        font: { weight: 'bold', size: 15 }
                    },
                    ticks: {
                        color: '#ccc',
                        font: { size: 14, weight: 'bold' },
                        maxTicksLimit: 12,
                        autoSkip: false,
                        callback: function(value, index, values) {
                            return index % 2 === 0 ? this.getLabelForValue(value) : '';
                        }
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                },
                y: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Success Rate (%)',
                        color: '#00ff88',
                        font: { weight: 'bold', size: 15 }
                    },
                    min: 90,
                    max: 100,
                    ticks: {
                        color: '#00ff88',
                        font: { size: 14, weight: 'bold' },
                        stepSize: 2,
                        callback: v => v.toFixed(0) + '%'
                    },
                    grid: {
                        display: true,
                        color: 'rgba(200,200,200,0.12)'
                    }
                }
            }
        }
    });
}

document.addEventListener('DOMContentLoaded', async () => {
    await loadTelemetry();
    updateDashboardWithTelemetry();
    renderTelemetryCharts();
    fetchAndDisplayLatestImage();
    setInterval(fetchAndDisplayLatestImage, 5000);
    // ...existing code for browseImageInput...
});

// Remove or comment out this line:
// updateDashboardWithTelemetry();

// Add some initial log entries based on telemetry data
addLogEntry(`System status: ${data.system_status}`, data.system_status === 'OFFLINE' ? 'error' : 'info');
addLogEntry(`Battery at ${data.eps.battery_percentage}% (${data.eps.battery_voltage}V)`, 'info');
addLogEntry(`${data.payload.status} - ${data.payload.images_today} images captured today`, 'info');
addLogEntry(`Last command: ${data.comms.last_command}`, 'info');
addLogEntry(`AI Classification: ${data.payload.ai_classification.result} (${data.payload.ai_classification.confidence}% confidence)`, 'info');

// Add random log entries periodically
setInterval(() => {
    const messages = [
        'Telemetry packet transmitted',
        'Image analysis completed', 
        'Battery voltage nominal',
        'ADCS pointing adjustment',
        'Ground station handshake'
    ];
    const message = messages[Math.floor(Math.random() * messages.length)];
    addLogEntry(message, 'info');
}, 5000);

// --- PAYLOAD IMAGE LOGIC (Backend Integration) ---
async function fetchAndDisplayLatestImage() {
    const imgElem = document.getElementById('payloadImage');
    if (!imgElem) return;
    try {
        // Fetch from Node.js backend, not VS Live Server
        const res = await fetch('http://localhost:3001/latest-image', { cache: "no-store" });
        if (!res.ok) throw new Error('No image');
        const data = await res.json();
        if (data.filename) {
            // Add a cache-busting query param to force reload
            const cacheBuster = Date.now();
            imgElem.src = `http://localhost:3001/images/${encodeURIComponent(data.filename)}?v=${cacheBuster}`;
            imgElem.style.width = '100%';
            imgElem.style.height = '100%';
            imgElem.style.objectFit = 'contain';
            imgElem.style.opacity = '1';
            // Remove placeholder if present
            let placeholder = document.getElementById('payloadImagePlaceholder');
            if (placeholder) placeholder.remove();
        } else {
            throw new Error('No image');
        }
    } catch (e) {
        // Show themed placeholder if no image
        const imgContainer = imgElem.parentNode;
        imgElem.src = '';
        imgElem.style.opacity = '0.2';
        // Remove old placeholder if any
        let placeholder = document.getElementById('payloadImagePlaceholder');
        if (!placeholder) {
            placeholder = document.createElement('div');
            placeholder.id = 'payloadImagePlaceholder';
            placeholder.style.position = 'absolute';
            placeholder.style.top = '0';
            placeholder.style.left = '0';
            placeholder.style.width = '100%';
            placeholder.style.height = '100%';
            placeholder.style.display = 'flex';
            placeholder.style.flexDirection = 'column';
            placeholder.style.alignItems = 'center';
            placeholder.style.justifyContent = 'center';
            placeholder.style.background = 'linear-gradient(45deg, #222 60%, #333 100%)';
            placeholder.style.color = '#00d4ff';
            placeholder.style.fontSize = '1.2rem';
            placeholder.style.fontWeight = 'bold';
            placeholder.style.zIndex = '2';
            placeholder.innerHTML = `
                <div style="font-size:2.5rem;opacity:0.7;">📷</div>
                <div style="margin-top:10px;">No new images</div>
            `;
            imgContainer.appendChild(placeholder);
        }
    }
}

// Optionally, allow manual refresh or poll every few seconds
document.addEventListener('DOMContentLoaded', () => {
    // ...existing code...
    fetchAndDisplayLatestImage();
    setInterval(fetchAndDisplayLatestImage, 5000);
    // ...existing code for browseImageInput...
});

// --- PAYLOAD IMAGE LOGIC (Browser Only) ---
document.addEventListener('DOMContentLoaded', () => {
    const browseInput = document.getElementById('browseImageInput');
    const browseInput2 = document.getElementById('browseImageInput2');
    const imgElem = document.getElementById('payloadImage');
    if (browseInput && imgElem) {
        browseInput.addEventListener('change', (e) => {
            const file = e.target.files && e.target.files[0];
            if (file) {
                const url = URL.createObjectURL(file);
                imgElem.src = url;
                imgElem.style.width = '100%';
                imgElem.style.height = '100%';
                imgElem.style.objectFit = 'contain';
            }
        });
    }
    // Add support for the second browse button
    if (browseInput2 && imgElem) {
        browseInput2.addEventListener('change', (e) => {
            const file = e.target.files && e.target.files[0];
            if (file) {
                const url = URL.createObjectURL(file);
                imgElem.src = url;
                imgElem.style.width = '100%';
                imgElem.style.height = '100%';
                imgElem.style.objectFit = 'contain';
            }
        });
    }
});

document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.control-btn').forEach(btn => {
        btn.addEventListener('click', e => {
            e.preventDefault();
        });
    });
});

console.log("dashboard.js loaded and sendCommand is global:", typeof window.sendCommand);