
function animateMouthWhileSpeaking() {
  const mouth = document.querySelector('.mouth');
  let mouthOpen = false;
  const mouthAnimationInterval = setInterval(() => {
      if (mouthOpen) {
          mouth.style.height = '20px';
          mouthOpen = false;
      } else {
          mouth.style.height = '40px';
          mouthOpen = true;
      }
  }, 300);

  audioPlayer.addEventListener('ended', () => {
    clearInterval(mouthAnimationInterval);
    mouth.style.height = '20px'; // Reset mouth state after speaking
  }, { once: true });
}

function resetMouth() {
  const mouth = document.querySelector('.mouth');
  mouth.style.height = '20px'; // Neutral position
}

function setEmotion(emotion) {
  const mouth = document.querySelector('.mouth');
  const leftEye = document.querySelector('.eye.left');
  const rightEye = document.querySelector('.eye.right');
  
  // Reset to neutral before setting a new emotion
  mouth.style.height = '20px';
  leftEye.style.transform = 'translateY(0)';
  rightEye.style.transform = 'translateY(0)';
  
  if (emotion === 'happy') {
      mouth.style.height = '30px'; // Wider smile
      mouth.style.background = '#4caf50'; // Green color
      leftEye.style.transform = 'translateY(-10px) rotate(10deg)';
      rightEye.style.transform = 'translateY(-10px) rotate(-10deg)';
  } else if (emotion === 'sad') {
      mouth.style.height = '10px'; // Frown
      mouth.style.background = '#1976d2'; // Blue color
      leftEye.style.transform = 'translateY(10px) rotate(10deg)';
      rightEye.style.transform = 'translateY(10px) rotate(-10deg)';
  } else if (emotion === 'angry') {
      mouth.style.height = '10px'; // Frown
      mouth.style.background = '#d32f2f'; // Red color
      leftEye.style.transform = 'translateY(-10px) rotate(10deg)';
      rightEye.style.transform = 'translateY(-10px) rotate(-10deg)';
  } else if (emotion === 'surprised') {
      mouth.style.height = '30px'; // Surprised
      mouth.style.background = '#ff4d4d'; // Red color
      leftEye.style.transform = 'translateY(-20px)';
      rightEye.style.transform = 'translateY(-20px)';
  }
}
