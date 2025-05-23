function updateStatus() {
    fetch('/system_status')
        .then(response => response.json())
        .then(data => {
           
            document.getElementById('ros-status').textContent = 
                data.ros_connected ? 'Connected' : 'Disconnected';
            document.getElementById('fps').textContent = 
                data.fps.toFixed(1) + ' FPS';
            document.getElementById('resolution').textContent = 
                data.resolution;
                 
            const recordBtn = document.getElementById('recordBtn');
            recordBtn.textContent = data.recording ? 
                'Stop Recording' : 'Start Recording';
            recordBtn.className = `control-btn${data.recording ? ' recording' : ''}`;

            const pauseBtn = document.getElementById('pauseBtn');
            pauseBtn.textContent = data.task_paused ? 
                'Resume Task' : 'Pause Task';
            pauseBtn.className = `control-btn${data.task_paused ? ' paused' : ''}`;
            
            const trackInfo = data.track_info || {};
            document.getElementById('track-id').textContent = trackInfo.id || 'N/A';
            document.getElementById('track-type').textContent = trackInfo.type || 'N/A';
        });
}

function toggleRecording() {
    const command = document.getElementById('recordBtn').textContent
        .includes('Start') ? 'start_recording' : 'stop_recording';
        
    sendControlCommand(command);
}

function togglePause() {
    const command = document.getElementById('pauseBtn').textContent
        .includes('Pause') ? 'pause_task' : 'resume_task';
        
    sendControlCommand(command);
}

function takeSnapshot() {
    sendControlCommand('snapshot');
}

function sendControlCommand(command) {
    fetch('/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({command: command})
    }).then(response => {
        if (!response.ok) {
            alert('Command failed');
        }
    });
}

setInterval(updateStatus, 1000);
updateStatus();
