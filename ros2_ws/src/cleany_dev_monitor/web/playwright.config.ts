import { defineConfig } from "@playwright/test";
export default defineConfig({
  webServer: process.env.MONITOR_URL
    ? undefined
    : {
        command: "pnpm exec vite --host 127.0.0.1 --port 4174 --strictPort",
        url: "http://127.0.0.1:4174",
        reuseExistingServer: false,
      },
  testDir: "./e2e",
  workers: 1,
  timeout: 30000,
  use: {
    baseURL: process.env.MONITOR_URL || "http://127.0.0.1:4174",
    viewport: { width: 1440, height: 1000 },
    launchOptions: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE
      ? { executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE }
      : {},
  },
  outputDir: "test-results",
});
