// Smooth scrolling for navigation links
document.addEventListener('DOMContentLoaded', function() {
    // Add intersection observer for animations
    const sections = document.querySelectorAll('.section');
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.style.opacity = '1';
                entry.target.style.transform = 'translateY(0)';
            }
        });
    }, observerOptions);

    sections.forEach(section => {
        observer.observe(section);
    });

    // Add hover effects to media placeholders
    const placeholders = document.querySelectorAll('.media-placeholder');
    placeholders.forEach(placeholder => {
        placeholder.addEventListener('mouseenter', function() {
            this.style.transform = 'scale(1.02)';
            this.style.transition = 'transform 0.3s ease';
        });

        placeholder.addEventListener('mouseleave', function() {
            this.style.transform = 'scale(1)';
        });
    });

    // Add copy functionality to citation box
    const citationBox = document.querySelector('.citation-box');
    if (citationBox) {
        citationBox.addEventListener('click', function() {
            const citationText = this.querySelector('code').textContent;
            
            // Try to copy to clipboard
            if (navigator.clipboard) {
                navigator.clipboard.writeText(citationText).then(() => {
                    showCopyNotification();
                }).catch(err => {
                    console.error('Failed to copy citation: ', err);
                });
            } else {
                // Fallback for older browsers
                const textArea = document.createElement('textarea');
                textArea.value = citationText;
                document.body.appendChild(textArea);
                textArea.select();
                try {
                    document.execCommand('copy');
                    showCopyNotification();
                } catch (err) {
                    console.error('Failed to copy citation: ', err);
                }
                document.body.removeChild(textArea);
            }
        });

        // Add cursor pointer to indicate clickability
        citationBox.style.cursor = 'pointer';
        citationBox.title = 'Click to copy citation';
    }

    // Smooth scroll to top functionality
    let scrollToTopButton = document.createElement('button');
    scrollToTopButton.innerHTML = '↑';
    scrollToTopButton.className = 'scroll-to-top';
    scrollToTopButton.style.cssText = `
        position: fixed;
        bottom: 30px;
        right: 30px;
        background-color: #0066cc;
        color: white;
        border: none;
        border-radius: 50%;
        width: 50px;
        height: 50px;
        font-size: 20px;
        cursor: pointer;
        opacity: 0;
        transition: opacity 0.3s ease, transform 0.3s ease;
        z-index: 1000;
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
    `;

    document.body.appendChild(scrollToTopButton);

    // Show/hide scroll to top button
    window.addEventListener('scroll', function() {
        if (window.pageYOffset > 300) {
            scrollToTopButton.style.opacity = '1';
            scrollToTopButton.style.transform = 'scale(1)';
        } else {
            scrollToTopButton.style.opacity = '0';
            scrollToTopButton.style.transform = 'scale(0.8)';
        }
    });

    // Scroll to top functionality
    scrollToTopButton.addEventListener('click', function() {
        window.scrollTo({
            top: 0,
            behavior: 'smooth'
        });
    });

    // Add hover effect to scroll button
    scrollToTopButton.addEventListener('mouseenter', function() {
        this.style.backgroundColor = '#004499';
        this.style.transform = 'scale(1.1)';
    });

    scrollToTopButton.addEventListener('mouseleave', function() {
        this.style.backgroundColor = '#0066cc';
        this.style.transform = 'scale(1)';
    });
});

// Function to show copy notification
function showCopyNotification() {
    const notification = document.createElement('div');
    notification.textContent = 'Citation copied to clipboard!';
    notification.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        background-color: #28a745;
        color: white;
        padding: 12px 20px;
        border-radius: 6px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        z-index: 1001;
        font-family: inherit;
        font-size: 14px;
        transition: opacity 0.3s ease;
    `;

    document.body.appendChild(notification);

    // Animate in
    setTimeout(() => {
        notification.style.opacity = '1';
    }, 10);

    // Remove after 3 seconds
    setTimeout(() => {
        notification.style.opacity = '0';
        setTimeout(() => {
            if (notification.parentNode) {
                document.body.removeChild(notification);
            }
        }, 300);
    }, 3000);
}

// Add loading animation
window.addEventListener('load', function() {
    document.body.style.opacity = '1';
    document.body.style.transition = 'opacity 0.5s ease';
});

// Preload any critical resources if needed
document.addEventListener('DOMContentLoaded', function() {
    // Add any preloading logic here if needed
    console.log('UniBiDex website loaded successfully!');
});
