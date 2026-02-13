const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('sensor data visible on ScienceTask', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-sensor-view-all')).toBeVisible({ timeout: 15000 });
});

test('view all opens modal', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  const viewAllBtn = page.getByTestId('pw-sensor-view-all');
  await expect(viewAllBtn).toBeVisible({ timeout: 15000 });
  await viewAllBtn.click();
  const modal = page.locator('.modal-backdrop.d-flex');
  await expect(modal).toBeVisible({ timeout: 5000 });
});

test('csv button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-sensor-csv-btn')).toBeVisible({ timeout: 15000 });
});
