document.addEventListener("DOMContentLoaded", () => {
    const btn = document.getElementById("copy-email");
    if (btn) {
        btn.addEventListener("click", (e) => {
            navigator.clipboard.writeText("km776@cornell.edu").then(() => {
                const tooltip = document.createElement("div");
                tooltip.innerText = "Copied!";
                tooltip.style.position = "fixed";
                tooltip.style.background = "#222";
                tooltip.style.color = "white";
                tooltip.style.padding = "10px 15px";
                tooltip.style.borderRadius = "6px";
                tooltip.style.boxShadow = "0 2px 8px rgba(0,0,0,0.3)";
                tooltip.style.fontSize = "16px";
                tooltip.style.fontWeight = "bold";
                tooltip.style.zIndex = 1000;
                tooltip.style.left = `${Math.min(window.innerWidth - 140, e.clientX + 10)}px`;
                tooltip.style.top = `${Math.max(10, e.clientY - 40)}px`;
                document.body.appendChild(tooltip);
                setTimeout(() => tooltip.remove(), 1500);
            });
        });
    }
});
