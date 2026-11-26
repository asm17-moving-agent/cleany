const menu = document.querySelector("#menu-toggle");
const navigation = document.querySelector("#site-nav");

function setMenu(open) {
  menu.setAttribute("aria-expanded", String(open));
  navigation.dataset.open = String(open);
}

menu.addEventListener("click", () => setMenu(navigation.dataset.open !== "true"));
navigation.addEventListener("click", (event) => {
  if (event.target.closest("a")) setMenu(false);
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && navigation.dataset.open === "true") {
    setMenu(false);
    menu.focus();
  }
});
document.addEventListener("pointerdown", (event) => {
  if (navigation.dataset.open === "true" && !event.target.closest(".header-inner")) setMenu(false);
});

const navLinks = Array.from(navigation.querySelectorAll("a"));
const navSections = navLinks.map((link) => document.querySelector(link.hash));
const sectionObserver = new IntersectionObserver((entries) => {
  for (const entry of entries) {
    const link = navLinks[navSections.indexOf(entry.target)];
    if (entry.isIntersecting) {
      navLinks.forEach((item) => item.removeAttribute("aria-current"));
      link.setAttribute("aria-current", "location");
    } else {
      link.removeAttribute("aria-current");
    }
  }
}, { rootMargin: "-20% 0px -60% 0px" });
navSections.forEach((section) => sectionObserver.observe(section));

const workflowTabs = document.querySelector(".workflow-tabs");
const mobileLayout = matchMedia("(max-width: 760px)");

function resetResponsiveControls() {
  workflowTabs.setAttribute("aria-orientation", mobileLayout.matches ? "horizontal" : "vertical");
  if (!mobileLayout.matches) setMenu(false);
}

resetResponsiveControls();
mobileLayout.addEventListener("change", resetResponsiveControls);

for (const tabList of document.querySelectorAll('[role="tablist"]')) {
  const tabs = Array.from(tabList.querySelectorAll('[role="tab"]'));

  function selectTab(activeTab) {
    for (const tab of tabs) {
      const active = tab === activeTab;
      tab.setAttribute("aria-selected", String(active));
      tab.tabIndex = active ? 0 : -1;
      document.getElementById(tab.getAttribute("aria-controls")).hidden = !active;
    }
  }

  tabList.addEventListener("click", (event) => {
    const tab = event.target.closest('[role="tab"]');
    if (tab) selectTab(tab);
  });
  tabList.addEventListener("keydown", (event) => {
    const index = tabs.indexOf(document.activeElement);
    if (index === -1) return;

    let next;
    const vertical = tabList.getAttribute("aria-orientation") === "vertical";
    const backward = vertical ? "ArrowUp" : "ArrowLeft";
    const forward = vertical ? "ArrowDown" : "ArrowRight";
    switch (event.key) {
      case "Home": next = 0; break;
      case "End": next = tabs.length - 1; break;
      case backward: next = (index + tabs.length - 1) % tabs.length; break;
      case forward: next = (index + 1) % tabs.length; break;
      default: return;
    }

    event.preventDefault();
    selectTab(tabs[next]);
    tabs[next].focus();
  });
}

const hotspots = document.querySelectorAll(".hotspot");

for (const hotspot of hotspots) {
  const details = document.getElementById(hotspot.getAttribute("aria-controls"));
  details.addEventListener("toggle", () => {
    hotspot.setAttribute("aria-pressed", String(details.open));
  });
  hotspot.addEventListener("click", () => {
    details.open = !details.open;
    if (details.open && mobileLayout.matches) {
      details.querySelector("summary").focus({ preventScroll: true });
      details.scrollIntoView({ block: "nearest" });
    }
  });
}

const comparison = document.querySelector("#desk-compare");
const comparisonRange = document.querySelector("#comparison-range");

function updateComparison() {
  const value = comparisonRange.value;
  comparison.style.setProperty("--compare-position", value + "%");
  comparisonRange.setAttribute("aria-valuetext", "작업 전 사진 " + value + "퍼센트");
}

comparisonRange.addEventListener("input", updateComparison);
updateComparison();

const videoTriggers = document.querySelectorAll("[data-video-dialog]");

for (const dialog of document.querySelectorAll(".demo-dialog")) {
  const video = dialog.querySelector("video");
  let dialogTrigger;

  for (const trigger of videoTriggers) {
    if (trigger.dataset.videoDialog !== dialog.id) continue;
    trigger.addEventListener("click", () => {
      dialogTrigger = trigger;
      dialog.showModal();
      video.play().catch(() => {
        // The native controls stay available if playback is blocked.
      });
    });
  }

  dialog.addEventListener("close", () => {
    video.pause();
    dialogTrigger?.focus({ preventScroll: true });
  });

  dialog.addEventListener("click", (event) => {
    if (event.target !== dialog) return;
    const bounds = dialog.getBoundingClientRect();
    if (event.clientX < bounds.left || event.clientX > bounds.right ||
        event.clientY < bounds.top || event.clientY > bounds.bottom) {
      dialog.close();
    }
  });
}
