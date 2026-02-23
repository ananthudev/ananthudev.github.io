/* =======================================================
   ANANTHU DEV — main.js
   GSAP animations · Nav · Typed · GLightbox · Portfolio
   ======================================================= */

(function () {
  'use strict';

  // -------------------------------------------------------
  // PRELOADER
  // -------------------------------------------------------
  window.addEventListener('load', () => {
    const preloader = document.getElementById('preloader');
    if (preloader) {
      preloader.style.opacity = '0';
      preloader.style.transition = 'opacity 0.5s ease';
      setTimeout(() => preloader.remove(), 500);
    }
  });

  // -------------------------------------------------------
  // FOOTER YEAR
  // -------------------------------------------------------
  const yearEl = document.getElementById('footerYear');
  if (yearEl) yearEl.textContent = new Date().getFullYear();

  // -------------------------------------------------------
  // CUSTOM CURSOR
  // -------------------------------------------------------
  const cursorDot     = document.getElementById('cursor-dot');
  const cursorOutline = document.getElementById('cursor-outline');

  if (cursorDot && cursorOutline) {
    let mouseX = 0, mouseY = 0;
    let outlineX = 0, outlineY = 0;

    document.addEventListener('mousemove', (e) => {
      mouseX = e.clientX;
      mouseY = e.clientY;
      cursorDot.style.left = mouseX + 'px';
      cursorDot.style.top  = mouseY + 'px';
    });

    (function animateOutline() {
      outlineX += (mouseX - outlineX) * 0.12;
      outlineY += (mouseY - outlineY) * 0.12;
      cursorOutline.style.left = outlineX + 'px';
      cursorOutline.style.top  = outlineY + 'px';
      requestAnimationFrame(animateOutline);
    })();
  }

  // -------------------------------------------------------
  // MOBILE NAV TOGGLE
  // -------------------------------------------------------
  const mobileToggle = document.getElementById('mobileNavToggle');
  const header       = document.getElementById('header');
  const navOverlay   = document.getElementById('nav-overlay');

  function openNav() {
    header.classList.add('nav-open');
    navOverlay.classList.add('visible');
    document.body.classList.add('nav-open');
    mobileToggle.innerHTML = '<i class="bi bi-x"></i>';
  }

  function closeNav() {
    header.classList.remove('nav-open');
    navOverlay.classList.remove('visible');
    document.body.classList.remove('nav-open');
    mobileToggle.innerHTML = '<i class="bi bi-list"></i>';
  }

  if (mobileToggle) {
    mobileToggle.addEventListener('click', () => {
      header.classList.contains('nav-open') ? closeNav() : openNav();
    });
  }

  if (navOverlay) {
    navOverlay.addEventListener('click', closeNav);
  }

  // -------------------------------------------------------
  // SMOOTH SCROLL + NAV CLOSE ON MOBILE
  // -------------------------------------------------------
  document.querySelectorAll('.scrollto').forEach(link => {
    link.addEventListener('click', function (e) {
      const targetId = this.getAttribute('href');
      if (!targetId || !targetId.startsWith('#')) return;
      const target = document.querySelector(targetId);
      if (!target) return;
      e.preventDefault();
      closeNav();
      target.scrollIntoView({ behavior: 'smooth', block: 'start' });
    });
  });

  // -------------------------------------------------------
  // ACTIVE NAV LINK ON SCROLL
  // -------------------------------------------------------
  const sections  = document.querySelectorAll('section[id]');
  const navLinks  = document.querySelectorAll('.nav-link');

  function updateActiveNav() {
    const scrollY = window.scrollY + 120;
    sections.forEach(section => {
      const top    = section.offsetTop;
      const height = section.offsetHeight;
      const id     = section.getAttribute('id');
      if (scrollY >= top && scrollY < top + height) {
        navLinks.forEach(l => l.classList.remove('active'));
        const active = document.querySelector(`.nav-link[href="#${id}"]`);
        if (active) active.classList.add('active');
      }
    });
  }

  window.addEventListener('scroll', updateActiveNav, { passive: true });

  // -------------------------------------------------------
  // BACK TO TOP BUTTON
  // -------------------------------------------------------
  const backToTop = document.getElementById('backToTop');
  if (backToTop) {
    window.addEventListener('scroll', () => {
      if (window.scrollY > 300) {
        backToTop.classList.add('visible');
      } else {
        backToTop.classList.remove('visible');
      }
    }, { passive: true });
  }

  // -------------------------------------------------------
  // TYPED.JS — Hero
  // -------------------------------------------------------
  const typedEl = document.querySelector('.typed');
  if (typedEl && typeof Typed !== 'undefined') {
    const items = typedEl.getAttribute('data-typed-items')
      .split(',').map(s => s.trim());
    new Typed('.typed', {
      strings: items,
      loop: true,
      typeSpeed: 80,
      backSpeed: 45,
      backDelay: 2200,
    });
  }

  // -------------------------------------------------------
  // GLIGHTBOX — Photography
  // -------------------------------------------------------
  if (typeof GLightbox !== 'undefined') {
    GLightbox({
      selector: '.portfolio-lightbox',
      touchNavigation: true,
      loop: true,
      zoomable: true,
    });
  }

  // -------------------------------------------------------
  // PORTFOLIO FILTER
  // -------------------------------------------------------
  const filterBtns  = document.querySelectorAll('.filter-btn');
  const masonryItems = document.querySelectorAll('.masonry-item');

  filterBtns.forEach(btn => {
    btn.addEventListener('click', function () {
      filterBtns.forEach(b => b.classList.remove('active'));
      this.classList.add('active');

      const filter = this.getAttribute('data-filter');

      masonryItems.forEach(item => {
        if (filter === 'all' || item.classList.contains(filter)) {
          item.classList.remove('hidden');
          item.style.animation = 'fadeIn 0.4s ease forwards';
        } else {
          item.classList.add('hidden');
        }
      });
    });
  });

  // -------------------------------------------------------
  // CONTACT FORM — mailto fallback
  // -------------------------------------------------------
  const contactForm = document.getElementById('contactForm');
  const formStatus  = document.getElementById('form-status');

  if (contactForm) {
    contactForm.addEventListener('submit', function (e) {
      e.preventDefault();
      const name    = this.name.value.trim();
      const email   = this.email.value.trim();
      const subject = this.subject.value.trim();
      const message = this.message.value.trim();

      const mailtoLink = `mailto:ananthudevdv@gmail.com`
        + `?subject=${encodeURIComponent(subject + ' — from ' + name)}`
        + `&body=${encodeURIComponent('Name: ' + name + '\nEmail: ' + email + '\n\n' + message)}`;

      window.location.href = mailtoLink;

      if (formStatus) {
        formStatus.textContent = 'Opening your email client…';
        formStatus.className = 'success';
        setTimeout(() => {
          formStatus.textContent = '';
          formStatus.className = '';
        }, 4000);
      }
    });
  }

  // -------------------------------------------------------
  // GSAP SCROLL ANIMATIONS
  // -------------------------------------------------------
  window.addEventListener('load', () => {
    if (typeof gsap === 'undefined' || typeof ScrollTrigger === 'undefined') return;

    gsap.registerPlugin(ScrollTrigger);

    // Generic reveal for .reveal-item elements
    gsap.utils.toArray('.reveal-item').forEach(el => {
      gsap.fromTo(el,
        { opacity: 0, y: 32 },
        {
          opacity: 1, y: 0,
          duration: 0.75,
          ease: 'power3.out',
          scrollTrigger: {
            trigger: el,
            start: 'top 88%',
            once: true,
          }
        }
      );
    });

    // Stagger timeline cards
    gsap.utils.toArray('.resume-col').forEach(col => {
      const items = col.querySelectorAll('.timeline-item');
      gsap.fromTo(items,
        { opacity: 0, x: -20 },
        {
          opacity: 1, x: 0,
          duration: 0.6,
          ease: 'power2.out',
          stagger: 0.12,
          scrollTrigger: {
            trigger: col,
            start: 'top 85%',
            once: true,
          }
        }
      );
    });

    // Masonry items stagger
    gsap.fromTo('.masonry-item',
      { opacity: 0, scale: 0.96 },
      {
        opacity: 1, scale: 1,
        duration: 0.5,
        ease: 'power2.out',
        stagger: { amount: 0.8, from: 'start' },
        scrollTrigger: {
          trigger: '.masonry-grid',
          start: 'top 85%',
          once: true,
        }
      }
    );

    // Skill bar fill animation
    const skillsSection = document.getElementById('skills');
    if (skillsSection) {
      ScrollTrigger.create({
        trigger: skillsSection,
        start: 'top 80%',
        once: true,
        onEnter: () => {
          document.querySelectorAll('.progress-fill').forEach(bar => {
            const targetWidth = bar.getAttribute('data-width') + '%';
            gsap.to(bar, {
              width: targetWidth,
              duration: 1.3,
              ease: 'power3.out',
              delay: 0.1,
            });
          });
        }
      });
    }

    // Hero entrance
    const heroTl = gsap.timeline({ defaults: { ease: 'power3.out' } });
    heroTl
      .from('.hero-eyebrow',  { opacity: 0, y: 20, duration: 0.6 }, 0.2)
      .from('.hero-name',     { opacity: 0, y: 30, duration: 0.7 }, 0.4)
      .from('.hero-typed-line', { opacity: 0, y: 20, duration: 0.6 }, 0.65)
      .from('.hero-sub',      { opacity: 0, y: 20, duration: 0.6 }, 0.8)
      .from('.hero-actions',  { opacity: 0, y: 20, duration: 0.5 }, 0.95);
  });

})();