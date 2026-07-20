import { expect, test } from "@playwright/test";

test("CSV workspace has responsive selectable plot grid and local run controls", async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 900 });
  await page.goto("/");
  await expect(page.getByRole("heading", { name: "Balancer" })).toBeVisible();
  await expect(page.getByRole("button", { name: "Start" })).toBeVisible();
  await expect(page.getByText(/History window/)).toBeVisible();
  await expect(page.locator("#navigator-window")).toBeVisible();
  await expect(page.getByRole("button", { name: "30 s" })).toHaveCount(0);
  await expect(page.getByRole("button", { name: "2 min" })).toHaveCount(0);
  await expect(page.locator("#window-duration")).toHaveText("15 s");
  await expect(page.locator("#plots")).toHaveAttribute("data-window-seconds", "15.00");
  await expect(page.locator("#plots canvas")).toHaveCount(4);
  const attitude = page.locator(".plot-card").filter({ has: page.getByRole("heading", { name: "Attitude" }) });
  const rate = page.locator(".plot-card").filter({ has: page.getByRole("heading", { name: "Pitch rate" }) });
  const motion = page.locator(".plot-card").filter({ has: page.getByRole("heading", { name: "Motion" }) });
  await expect(attitude.locator(".legend")).not.toContainText("Accel");
  await expect(attitude.locator(".legend")).not.toContainText("Fused");
  await expect(rate.locator(".legend")).toContainText("Gyro");
  await expect(rate.locator(".legend")).not.toContainText("Measured");
  await expect(rate.locator(".legend")).not.toContainText("Filtered");
  await expect(motion.locator(".legend")).toContainText("Command");
  await expect(motion.locator(".legend")).toContainText("Left applied");
  await expect(motion.locator(".legend")).toContainText("Right applied");
  await expect(motion.locator(".legend")).not.toContainText("Left target");
  await page.getByRole("button", { name: "Command" }).click();
  await expect(page.locator("#plots canvas")).toHaveCount(5);
  await page.getByRole("button", { name: "Start" }).click();
  await expect(page.locator("#connection")).toHaveText("Playing CSV");
  await page.getByRole("button", { name: "Abort" }).click();
  await expect(page.locator("#connection")).toHaveText("CSV ready");
  const chart = page.locator(".plot-card").first();
  const box = await chart.boundingBox();
  expect(box?.width).toBeGreaterThan(500);
  expect(box?.height).toBeGreaterThan(300);
});

test("a new live run keeps a partially populated fixed 0 to 15 second axis", async ({ page }) => {
  const sample = {
    sequence: 1, received_at: 100, t_sec: 0,
    attitude: { pitch_deg: 1, fused_pitch_deg: 1, raw_acc_pitch_deg: 1, pitch_setpoint_deg: 0 },
    rate: { pitch_rate_dps: 0, filtered_pitch_rate_dps: 0, gyro_pitch_rate_dps: 0, rate_setpoint_dps: 0 },
    motion: { target_velocity_sps: 0, measured_velocity_sps: 0, left_target_sps: 0, right_target_sps: 0 },
    controller: { command_sps: 0, velocity_error: 0, pitch_error_deg: 0, velocity_p_term_deg: 0, velocity_i_term_deg: 0 },
    timing: { imu_age_ms: 1, feedback_age_ms: 2 }, flags: { controller: 0, saturation: 0, actuator: 0 },
  };
  await page.route("**/api/source", route => route.fulfill({ json: { mode:"live", name:"rpi4", duration_s:null, configured_pi:"rpi4", connection_state:"streaming", connection_message:"", run_limit_s:120, display_sample_hz:50, display_run:3 } }));
  await page.route("**/api/history?*", route => route.fulfill({ json: { samples:[sample], earliest:100, latest:100, window_start:100, window_end:100, cached_start:100, cached_end:100, decimated:false, run_limit_s:120, display_sample_hz:50, display_run:3 } }));
  await page.goto("/");
  await expect(page.locator("#plots")).toHaveAttribute("data-view-start", "0.00");
  await expect(page.locator("#plots")).toHaveAttribute("data-view-end", "15.00");
  await expect(page.locator("#navigator-fill")).toHaveAttribute("style", /width:0%/);
});

test("navigator resizes and pans the shared fixed window from the browser cache", async ({ page }) => {
  let historyRequests = 0;
  page.on("request", request => { if (request.url().includes("/api/history")) historyRequests++; });
  await page.goto("/");
  await expect(page.locator("body")).toHaveAttribute("data-cache-ready", "true");
  const track = page.locator("#navigator-track");
  const rightHandle = page.locator(".right-handle");
  const trackBox = await track.boundingBox();
  const handleBox = await rightHandle.boundingBox();
  expect(trackBox).not.toBeNull(); expect(handleBox).not.toBeNull();
  const requestsBeforeResize = historyRequests;
  await page.mouse.move(handleBox!.x + handleBox!.width / 2, handleBox!.y + handleBox!.height / 2);
  await page.mouse.down();
  await page.mouse.move(handleBox!.x - trackBox!.width / 4, handleBox!.y + handleBox!.height / 2, { steps: 8 });
  await expect(page.locator("#plots")).not.toHaveAttribute("data-window-seconds", "15.00");
  expect(historyRequests).toBe(requestsBeforeResize);
  await page.mouse.up();
  expect(historyRequests).toBe(requestsBeforeResize);

  const resizedHandle = await rightHandle.boundingBox();
  await page.mouse.move(resizedHandle!.x + resizedHandle!.width / 2, resizedHandle!.y + resizedHandle!.height / 2);
  await page.mouse.down();
  await page.mouse.move(trackBox!.x, resizedHandle!.y + resizedHandle!.height / 2, { steps: 8 });
  await expect(page.locator("#plots")).toHaveAttribute("data-window-seconds", "5.00");
  await page.mouse.up();
  const navigatorLabel = page.locator("#navigator-window span");
  expect(await navigatorLabel.evaluate(element => element.scrollHeight <= element.clientHeight)).toBe(true);

  const windowBox = await page.locator("#navigator-window").boundingBox();
  const beforePan = await page.locator("#plots").getAttribute("data-view-start");
  await page.mouse.move(windowBox!.x + windowBox!.width / 2, windowBox!.y + windowBox!.height / 2);
  await page.mouse.down();
  await page.mouse.move(windowBox!.x + windowBox!.width / 2 + 80, windowBox!.y + windowBox!.height / 2, { steps: 6 });
  await page.mouse.up();
  await expect(page.locator("#plots")).not.toHaveAttribute("data-view-start", beforePan!);
  await page.getByRole("button", { name: "Follow latest" }).click();
  await expect(page.locator("#timeline-state")).toHaveText("Following latest");
});

test("cursor readouts use stable columns and two-decimal elapsed time", async ({ page }) => {
  await page.goto("/");
  const chart = page.locator(".chart").first();
  const box = await chart.boundingBox();
  await page.mouse.move(box!.x + box!.width / 2, box!.y + box!.height / 2);
  await expect(page.locator(".cursor-time").first()).toHaveText(/^\d+\.\d{2} s$/);
  const value = page.locator(".cursor-value output").first();
  await expect(value).toBeVisible();
  expect(await value.evaluate(element => getComputedStyle(element).textAlign)).toBe("right");
  expect(await value.evaluate(element => getComputedStyle(element).fontVariantNumeric)).toContain("tabular-nums");
  const readouts = page.locator(".cursor-value");
  for (let index=0; index<await readouts.count(); index++) {
    expect(await readouts.nth(index).evaluate(element => element.scrollWidth <= element.clientWidth)).toBe(true);
  }
});

test("run controls show and enforce an immediate busy state", async ({ page }) => {
  await page.route("**/api/start", async route => {
    await new Promise(resolve => setTimeout(resolve, 500));
    await route.fulfill({ json: { ok: true, message: "Started.", display_run: 1 } });
  });
  await page.goto("/");
  await expect(page.locator("#source")).toContainText("CSV");
  const start = page.getByRole("button", { name: "Start" });
  await start.click();
  await expect(page.locator("#operation-result")).toHaveText("Starting CSV playback…");
  await expect(start).toBeDisabled();
  await expect(page.getByRole("button", { name: "Abort" })).toBeDisabled();
  await expect(page.locator("#operation-result")).toHaveText("Started.");
  await expect(start).toBeEnabled();
});
