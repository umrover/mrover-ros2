const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('bell button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-notification-bell')).toBeVisible({ timeout: 10000 });
});

test('clicking bell opens panel', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const bell = page.getByTestId('pw-notification-bell');
  await expect(bell).toBeVisible({ timeout: 10000 });
  await bell.click();
  await expect(page.getByTestId('pw-notification-panel')).toBeVisible({ timeout: 5000 });
});

test('clicking bell again closes panel', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const bell = page.getByTestId('pw-notification-bell');
  await expect(bell).toBeVisible({ timeout: 10000 });

  await bell.click();
  await expect(page.getByTestId('pw-notification-panel')).toBeVisible({ timeout: 5000 });

  const backdrop = page.locator('.position-fixed.top-0.start-0.w-100.h-100');
  await backdrop.click({ position: { x: 10, y: 10 } });
  await expect(page.getByTestId('pw-notification-panel')).not.toBeVisible({ timeout: 5000 });
});
