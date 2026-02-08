const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('theme dropdown opens', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await expect(themeBtn).toBeVisible({ timeout: 10000 });
  await themeBtn.click();
  const dropdown = page.locator('.dropdown-menu.show');
  await expect(dropdown).toBeVisible({ timeout: 5000 });
});

test('dark theme applies', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await themeBtn.click();
  const dropdown = page.locator('.dropdown-menu.show');
  await expect(dropdown).toBeVisible({ timeout: 5000 });
  await dropdown.locator('.dropdown-item').nth(1).click();
  const html = page.locator('html');
  await expect(html).toHaveAttribute('data-bs-theme', 'dark', { timeout: 5000 });
});

test('light theme applies', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await themeBtn.click();
  const dropdown = page.locator('.dropdown-menu.show');
  await expect(dropdown).toBeVisible({ timeout: 5000 });
  await dropdown.locator('.dropdown-item').nth(0).click();
  const html = page.locator('html');
  await expect(html).toHaveAttribute('data-bs-theme', 'light', { timeout: 5000 });
});

test('grid lock toggles on DMTask', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const lockBtn = page.getByTestId('pw-grid-lock-btn');
  await expect(lockBtn).toBeVisible({ timeout: 15000 });
  const initialClass = await lockBtn.getAttribute('class');
  await lockBtn.click();
  await page.waitForTimeout(500);
  const newClass = await lockBtn.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('grid controls hidden on home', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await page.waitForLoadState('networkidle');
  const lockBtn = page.getByTestId('pw-grid-lock-btn');
  await expect(lockBtn).not.toBeVisible({ timeout: 3000 });
});

test('logo links home', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const logo = page.locator('.logo');
  await expect(logo).toBeVisible({ timeout: 10000 });
  await logo.click();
  await expect(page).toHaveURL(/\/$/);
});
