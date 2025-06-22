
const speechButton = document.getElementById('speech-button');
const recognizedTextDiv = document.getElementById('recognized-text');
const audioPlayer = document.getElementById('audio-player');
let isRecognizing = false;
let isProcessing = false;

const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;

if (SpeechRecognition) {
    const recognition = new SpeechRecognition();
    recognition.lang = 'en-US';
    recognition.interimResults = true;
    recognition.maxAlternatives = 1;

    function updateButtonStyle() {
        if (isRecognizing) {
            speechButton.textContent = "Stop Speaking";
            speechButton.classList.remove('start-speaking');
            speechButton.classList.add('stop-speaking');
        } else {
            speechButton.textContent = "Start Speaking";
            speechButton.classList.remove('stop-speaking');
            speechButton.classList.add('start-speaking');
        }
    }

    function startRecognition() {
        if (isRecognizing) recognition.stop();
        else recognition.start();
        updateButtonStyle();
    }

    recognition.onstart = () => {
        isRecognizing = true;
        updateButtonStyle();
    };

    recognition.onspeechend = () => {
        if (isRecognizing) {
            recognition.stop();
            speechButton.textContent = "Start Speaking";
            isRecognizing = false;
            updateButtonStyle();
            sendToServer(recognizedTextDiv.textContent);
        }
    };

    recognition.onerror = (event) => {
        console.error("Speech recognition error:", event.error);
        recognizedTextDiv.textContent = `Error: ${event.error}. Please try again.`;
        speechButton.textContent = "Start Speaking";
        isRecognizing = false;
        updateButtonStyle();
    };

    recognition.onresult = (event) => {
        const transcript = event.results[event.resultIndex][0].transcript;
        recognizedTextDiv.textContent = transcript;
    };

    speechButton.onclick = () => {
        if (isRecognizing) {
            recognition.stop();
            isRecognizing = false;
            updateButtonStyle();
        } else {
            startRecognition();
        }
    };
} else {
    recognizedTextDiv.textContent = "Speech recognition is not supported in this browser.";
}

async function sendToServer(text) {
    const formData = new FormData();
    formData.append('text', text);

    if (!text.trim()) return; // No text to send.

    try {
        if (isProcessing) return; // Skip if already processing.
        isProcessing = true;

        // Step 1: Send text to /qa and get the response
        const qaResponse = await fetch('/qa', { method: 'POST', body: formData });
        if (!qaResponse.ok) {
            alert("Error getting QA response.");
            isProcessing = false;
            return;
        }

        const qaData = await qaResponse.json();
        const qaText = qaData.response; // Extract the response text

        if (!qaText.trim()) {
            alert("QA response is empty.");
            isProcessing = false;
            return;
        }

        // Step 2: Send the QA response to /tts for speech generation
        const ttsFormData = new FormData();
        ttsFormData.append('text', qaText);

        const ttsResponse = await fetch('/tts', { method: 'POST', body: ttsFormData });
        if (ttsResponse.ok) {
            setEmotion('sad');
            const timestamp = new Date().getTime();
            audioPlayer.src = `/static/output.mp3?t=${timestamp}`;
            audioPlayer.play();
            animateMouthWhileSpeaking();

            // Handle when audio ends
            audioPlayer.onended = () => {
                startRecognition();
                speechButton.textContent = "Stop Speaking";
                isProcessing = false;
                resetMouth();
            };
        } else {
            alert("Error generating speech.");
            isProcessing = false;
            resetMouth();
        }
    } catch (error) {
        console.error('Error:', error);
        recognizedTextDiv.textContent = "Error processing speech.";
        isProcessing = false;
        resetMouth();
    }
}


document.getElementById("throw-button").addEventListener("click", async (e) => {
    e.preventDefault();
    try {
        const response = await fetch("/throw", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ action: "throwStone" }) });
        if (response.ok) {
            const result = await response.json();
            alert("Stone thrown successfully!");
        } else {
            alert("Failed to throw stone.");
        }
    } catch (err) {
        alert("Error occurred.");
    }
});
