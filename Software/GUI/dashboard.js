import { data } from "./telemetry.js";

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
    
    // Update light sensors
    document.getElementById('lightSensorA').textContent = data.adcs.light_sensors.face_a;
    document.getElementById('lightSensorB').textContent = data.adcs.light_sensors.face_b;
    
    // Update EPS data
    document.getElementById('batteryVoltage').textContent = data.eps.battery_voltage + 'V';
    document.getElementById('batteryPercent').textContent = data.eps.battery_percentage + '%';
    document.getElementById('batteryPercentage').textContent = data.eps.battery_percentage + '%';
    document.getElementById('current').textContent = data.eps.current + 'A';
    document.getElementById('solarPower').textContent = data.eps.solar_power + 'W';
    document.getElementById('temperature').textContent = data.eps.temperature + '°C';
    document.getElementById('systemTemp').textContent = data.eps.temperature + '°C';
    
    // Update battery progress bars
    const batteryBar = document.getElementById('batteryBar');
    const batteryProgress = document.getElementById('batteryProgress');
    batteryBar.style.width = data.eps.battery_percentage + '%';
    batteryProgress.style.width = data.eps.battery_percentage + '%';
    
    // Set battery color based on percentage
    let batteryColor;
    if (data.eps.battery_percentage <= 20) {
        batteryColor = '#ff4444';
    } else if (data.eps.battery_percentage <= 60) {
        batteryColor = '#ffaa00';
    } else {
        batteryColor = '#00ff88';
    }
    batteryBar.style.backgroundColor = batteryColor;
    batteryProgress.style.background = `linear-gradient(90deg, ${batteryColor}, ${batteryColor})`;
    
    // Update temperature bar
    const tempPercent = Math.round(((data.eps.temperature - 15) / 70) * 100);
    const tempLine = document.querySelector('.temp-line');
    if (tempLine) {
        tempLine.style.width = tempPercent + '%';
        let tempColor = tempPercent <= 50 ? '#00ff88' : '#ff4444';
        tempLine.style.background = `linear-gradient(to top, transparent, ${tempColor})`;
    }
    
    // Update OBC data
    document.getElementById('storageUsage').textContent = data.obc.storage_usage + '%';
    document.getElementById('storageProgress').style.width = data.obc.storage_usage + '%';
    document.getElementById('commandQueue').textContent = data.obc.command_queue;
    
    // Update COMMS data
    document.getElementById('packetsSent').textContent = data.comms.packets_sent.toLocaleString();
    document.getElementById('packetsReceived').textContent = data.comms.packets_received.toLocaleString();
    document.getElementById('signalStrength').textContent = data.comms.signal_strength + ' dBm';
    document.getElementById('lastCommand').textContent = `Last: "${data.comms.last_command}" - ${data.timestamp.split(' ')[1]}`;
    
    // Update Payload data
    const payloadStatus = document.getElementById('payloadStatus');
    payloadStatus.textContent = data.payload.status;
    payloadStatus.style.color = data.payload.status === 'CAPTURING' ? '#ffaa00' : '#00ff88';
    
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
        adcsStatus.className = 'status-indicator status-online';
        epsStatus.className = 'status-indicator status-online';
        obcStatus.className = 'status-indicator status-online';
        commsStatus.className = 'status-indicator status-online';
        payloadSubsystemStatus.className = data.payload.status === 'CAPTURING' 
            ? 'status-indicator status-warning' 
            : 'status-indicator status-online';
    }
    
}

function sendCommand(command) {
    const statusElement = document.getElementById('commandStatus');
    statusElement.textContent = `Sending ${command} command...`;
    statusElement.style.color = '#ffaa00';

    setTimeout(() => {
        statusElement.textContent = `${command} command sent successfully!`;
        statusElement.style.color = '#00ff88';
        addLogEntry(`Command sent: ${command}`, 'info');
    }, 1000);
}

function emergencyStop() {
    if (confirm('Are you sure you want to execute EMERGENCY STOP?')) {
        addLogEntry('EMERGENCY STOP executed by ground control', 'error');
        document.getElementById('commandStatus').textContent = 'EMERGENCY STOP ACTIVE';
        document.getElementById('commandStatus').style.color = '#ff4444';
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

// Make functions global so they can be called from HTML
window.sendCommand = sendCommand;
window.emergencyStop = emergencyStop;

// Initialize dashboard with telemetry data
updateDashboardWithTelemetry();

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