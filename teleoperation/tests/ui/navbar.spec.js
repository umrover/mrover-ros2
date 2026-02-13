const { test, expect } = require('@playwright/test');

test('theme dropdown opens', async ({ page }) => {
  await page.goto('/');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await expect(themeBtn).toBeVisible();
  await themeBtn.click();
  const dropdown = page.locator('.cmd-dropdown-menu.show');
  await expect(dropdown).toBeVisible();
});

test('dark theme applies', async ({ page }) => {
  await page.goto('/');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await themeBtn.click();
  const dropdown = page.locator('.cmd-dropdown-menu.show');
  await expect(dropdown).toBeVisible();
  await dropdown.locator('.cmd-dropdown-item').nth(1).click();
  const html = page.locator('html');
  await expect(html).toHaveAttribute('data-theme', 'dark');
});

test('light theme applies', async ({ page }) => {
  await page.goto('/');
  const themeBtn = page.getByTestId('pw-theme-dropdown');
  await themeBtn.click();
  const dropdown = page.locator('.cmd-dropdown-menu.show');
  await expect(dropdown).toBeVisible();
  await dropdown.locator('.cmd-dropdown-item').nth(0).click();
  const html = page.locator('html');
  await expect(html).toHaveAttribute('data-theme', 'light');
});

test('grid lock toggles on DMTask', async ({ page }) => {
  await page.goto('/DMTask');
  const lockBtn = page.getByTestId('pw-grid-lock-btn');
  await expect(lockBtn).toBeVisible();
  const initialClass = await lockBtn.getAttribute('class');
  await lockBtn.click();
  await page.waitForTimeout(300);
  const newClass = await lockBtn.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('grid controls hidden on home', async ({ page }) => {
  await page.goto('/');
  const lockBtn = page.getByTestId('pw-grid-lock-btn');
  await expect(lockBtn).not.toBeVisible({ timeout: 3000 });
});

test('logo links home', async ({ page }) => {
  await page.goto('/DMTask');
  const logo = page.locator('.logo');
  await expect(logo).toBeVisible();
  await logo.click();
  await expect(page).toHaveURL(/\/$/);
});
