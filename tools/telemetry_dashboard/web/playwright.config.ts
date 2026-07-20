import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  use: { baseURL: "http://127.0.0.1:8123", browserName: "chromium", headless: true },
  webServer: {
    command: "python3 tools/telemetry_dashboard/server.py --csv tools/telemetry_dashboard/web/tests/fixtures/telemetry.csv --playback-speed 0 --port 8123",
    cwd: "../../..",
    reuseExistingServer: true,
  },
});
