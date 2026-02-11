const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('frontend loads with title', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await expect(page).toHaveTitle(/MRover/);
});

test('4 mission buttons visible', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const buttons = page.getByTestId('pw-menu-btn');
  await expect(buttons).toHaveCount(4);
});

test('Delivery navigates to /DMTask', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Delivery' });
  await btn.click();
  await expect(page).toHaveURL(/\/DMTask/);
});

test('Equipment Servicing navigates to /ESTask', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Equipment Servicing' });
  await btn.click();
  await expect(page).toHaveURL(/\/ESTask/);
});

test('Science navigates to /ScienceTask', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Science' });
  await btn.click();
  await expect(page).toHaveURL(/\/ScienceTask/);
});

test('Autonomy navigates to /AutonTask', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Autonomy' });
  await btn.click();
  await expect(page).toHaveURL(/\/AutonTask/);
});
