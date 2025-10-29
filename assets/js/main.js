// FTH - Formação Tecnológica em Humanoides
// Main JavaScript File

// Navigation Menu Toggle (Mobile)
document.addEventListener('DOMContentLoaded', function() {
  const navTrigger = document.getElementById('nav-trigger');
  const navLinks = document.querySelector('.nav-links');
  
  if (navTrigger && navLinks) {
    navTrigger.addEventListener('change', function() {
      if (this.checked) {
        navLinks.classList.add('active');
      } else {
        navLinks.classList.remove('active');
      }
    });
  }
});

// Progress Tracking System
const ProgressTracker = {
  storageKey: 'fth-progress',
  
  getCompleted: function() {
    const data = localStorage.getItem(this.storageKey);
    return data ? JSON.parse(data) : [];
  },
  
  markComplete: function(moduleId) {
    let completed = this.getCompleted();
    if (!completed.includes(moduleId)) {
      completed.push(moduleId);
      localStorage.setItem(this.storageKey, JSON.stringify(completed));
      return true;
    }
    return false;
  },
  
  getProgress: function(totalModules = 12) {
    const completed = this.getCompleted().length;
    return Math.round((completed / totalModules) * 100);
  }
};

// Update Progress Display
function updateProgressDisplay() {
  const progressBar = document.getElementById('progress-bar');
  if (progressBar) {
    const progress = ProgressTracker.getProgress();
    progressBar.style.width = progress + '%';
  }
}

// Initialize
window.addEventListener('DOMContentLoaded', updateProgressDisplay);
